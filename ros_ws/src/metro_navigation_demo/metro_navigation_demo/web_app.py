import asyncio
import secrets
from pathlib import Path
from typing import List, Literal, Optional

from fastapi import Depends, FastAPI, Header, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field, root_validator
from websockets.exceptions import ConnectionClosed

from .command_gateway import CommandGateway
from .defect_store import DefectStore
from .route_config import RouteCatalog
from .status_store import StatusStore
from .camera_profiles import camera_config


class NavigationStart(BaseModel):
    mode: Literal['junction', 'destination']
    destination: Optional[str] = Field(default=None, max_length=64)


class JunctionChoice(BaseModel):
    choice: str = Field(min_length=1, max_length=64)


class CameraSelection(BaseModel):
    camera_name: str = Field(min_length=1, max_length=32)


class Position(BaseModel):
    x: float = Field(ge=-100000.0, le=100000.0)
    y: float = Field(ge=-100000.0, le=100000.0)
    z: float = Field(ge=-10000.0, le=10000.0)


class BoundingBox(BaseModel):
    x: float = Field(ge=0.0)
    y: float = Field(ge=0.0)
    width: float = Field(gt=0.0)
    height: float = Field(gt=0.0)
    image_width: int = Field(gt=0, le=20000)
    image_height: int = Field(gt=0, le=20000)

    @root_validator
    def image_bounds(cls, values: dict) -> dict:
        x = values.get('x')
        y = values.get('y')
        width = values.get('width')
        height = values.get('height')
        image_width = values.get('image_width')
        image_height = values.get('image_height')
        if None not in (x, width, image_width) and x + width > image_width:
            raise ValueError('bbox exceeds image width')
        if None not in (y, height, image_height) and y + height > image_height:
            raise ValueError('bbox exceeds image height')
        return values


class DefectCreate(BaseModel):
    type: str = Field(min_length=1, max_length=80)
    level: Literal['I级 (轻微)', 'II级 (中度)', 'III级 (严重)']
    mileage: str = Field(min_length=1, max_length=40)
    ring_number: int = Field(ge=0, le=10000000)
    clock_position: float = Field(ge=0.0, le=12.0)
    position: Position
    confidence: Optional[float] = Field(default=None, ge=0.0, le=1.0)
    camera_name: Optional[str] = Field(default=None, min_length=1, max_length=32)
    bbox: Optional[BoundingBox] = None


def create_app(
    status_store: StatusStore,
    dashboard_dir: Path,
    command_gateway: CommandGateway,
    routes: RouteCatalog,
    defect_store: DefectStore,
    camera_directory: Path,
) -> FastAPI:
    dashboard_dir = dashboard_dir.resolve()
    camera_directory = camera_directory.resolve()
    if not (dashboard_dir / 'index.html').is_file():
        raise FileNotFoundError(f'Dashboard index not found: {dashboard_dir / "index.html"}')

    session_token = secrets.token_urlsafe(32)
    cameras = camera_config()
    camera_names = frozenset(camera['id'] for camera in cameras)
    app = FastAPI(
        title='Subway Patrol Web Bridge',
        version='1.0.0',
        docs_url=None,
        redoc_url=None,
        openapi_url=None,
    )

    async def require_session_token(
        x_session_token: Optional[str] = Header(default=None, alias='X-Session-Token'),
    ) -> None:
        if x_session_token is None or not secrets.compare_digest(
            x_session_token, session_token
        ):
            raise HTTPException(status_code=403, detail='invalid session token')

    def dispatch(command: dict) -> JSONResponse:
        try:
            command_gateway.send(command)
        except RuntimeError as error:
            raise HTTPException(status_code=503, detail=str(error)) from error
        return JSONResponse({'accepted': True, 'command': command['command']})

    @app.get('/api/health')
    @app.get('/api/status')
    async def status() -> JSONResponse:
        return JSONResponse(
            content=status_store.snapshot(), headers={'Cache-Control': 'no-store'}
        )

    @app.get('/api/session')
    async def session() -> JSONResponse:
        return JSONResponse(
            {'token': session_token}, headers={'Cache-Control': 'no-store'}
        )

    @app.get('/api/navigation/options')
    async def navigation_options() -> JSONResponse:
        return JSONResponse(routes.public_options(), headers={'Cache-Control': 'no-store'})

    @app.get('/api/cameras')
    async def camera_options() -> JSONResponse:
        return JSONResponse({'cameras': cameras}, headers={'Cache-Control': 'no-store'})

    @app.post('/api/session/heartbeat', dependencies=[Depends(require_session_token)])
    async def heartbeat() -> JSONResponse:
        return dispatch({'command': 'heartbeat'})

    @app.post('/api/navigation/start', dependencies=[Depends(require_session_token)])
    async def start_navigation(request: NavigationStart) -> JSONResponse:
        if request.mode == 'destination':
            if request.destination is None or not routes.has_destination(request.destination):
                raise HTTPException(status_code=422, detail='unknown destination')
        return dispatch({
            'command': 'start',
            'mode': request.mode,
            'destination': request.destination,
        })

    @app.post(
        '/api/navigation/junction-choice',
        dependencies=[Depends(require_session_token)],
    )
    async def junction_choice(request: JunctionChoice) -> JSONResponse:
        if not routes.has_junction_choice(request.choice):
            raise HTTPException(status_code=422, detail='unknown junction choice')
        return dispatch({'command': 'junction_choice', 'choice': request.choice})

    @app.post('/api/navigation/pause', dependencies=[Depends(require_session_token)])
    async def pause_navigation() -> JSONResponse:
        return dispatch({'command': 'pause'})

    @app.post('/api/navigation/resume', dependencies=[Depends(require_session_token)])
    async def resume_navigation() -> JSONResponse:
        return dispatch({'command': 'resume'})

    @app.post('/api/navigation/cancel', dependencies=[Depends(require_session_token)])
    async def cancel_navigation() -> JSONResponse:
        return dispatch({'command': 'cancel'})

    @app.post('/api/safety/estop', dependencies=[Depends(require_session_token)])
    async def activate_estop() -> JSONResponse:
        return dispatch({'command': 'estop'})

    @app.post('/api/safety/reset', dependencies=[Depends(require_session_token)])
    async def reset_estop() -> JSONResponse:
        return dispatch({'command': 'reset_estop'})

    @app.get('/api/defects')
    async def list_defects() -> JSONResponse:
        return JSONResponse(
            {'records': defect_store.list()}, headers={'Cache-Control': 'no-store'}
        )

    @app.post('/api/defects', dependencies=[Depends(require_session_token)])
    async def add_defect(request: DefectCreate) -> JSONResponse:
        if request.camera_name is not None and request.camera_name not in camera_names:
            raise HTTPException(status_code=422, detail='unknown camera')
        stored = defect_store.add(request.dict())
        return JSONResponse(stored, status_code=201)

    def camera_path(camera_name: str) -> Path:
        if camera_name not in camera_names:
            raise HTTPException(status_code=404, detail='unknown camera')
        return camera_directory / f'{camera_name}.jpg'

    @app.get('/api/camera/{camera_name}.jpg')
    async def camera_jpeg(camera_name: str) -> FileResponse:
        path = camera_path(camera_name)
        if not path.is_file():
            raise HTTPException(status_code=404, detail='camera frame unavailable')
        return FileResponse(
            path,
            media_type='image/jpeg',
            headers={'Cache-Control': 'no-store, max-age=0'},
        )

    @app.post('/api/camera/select', dependencies=[Depends(require_session_token)])
    async def select_camera(request: CameraSelection) -> JSONResponse:
        if request.camera_name not in camera_names:
            raise HTTPException(status_code=422, detail='unknown camera')
        return dispatch({
            'command': 'select_camera',
            'camera_name': request.camera_name,
        })

    @app.get('/api/camera/{camera_name}.mjpeg')
    async def camera_mjpeg(camera_name: str) -> StreamingResponse:
        path = camera_path(camera_name)

        async def frames():
            last_modified = -1
            while True:
                try:
                    modified = path.stat().st_mtime_ns
                    if modified != last_modified:
                        frame = path.read_bytes()
                        if frame:
                            yield (
                                b'--frame\r\nContent-Type: image/jpeg\r\n'
                                + f'Content-Length: {len(frame)}\r\n\r\n'.encode('ascii')
                                + frame
                                + b'\r\n'
                            )
                            last_modified = modified
                except FileNotFoundError:
                    pass
                await asyncio.sleep(0.12)

        return StreamingResponse(
            frames(),
            media_type='multipart/x-mixed-replace; boundary=frame',
            headers={'Cache-Control': 'no-store, max-age=0'},
        )

    @app.websocket('/ws/status')
    async def websocket_status(websocket: WebSocket) -> None:
        await websocket.accept()
        try:
            while True:
                await websocket.send_json(status_store.snapshot())
                await asyncio.sleep(0.25)
        except (ConnectionClosed, WebSocketDisconnect, RuntimeError):
            return

    app.mount(
        '/',
        StaticFiles(directory=str(dashboard_dir), html=True, follow_symlink=True),
        name='inspection_dashboard',
    )
    return app
