from pathlib import Path
from typing import Callable, Dict, Optional

from fastapi import FastAPI, HTTPException, Query, WebSocket
from fastapi.responses import JSONResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles

from .camera_store import CameraFrameStore
from .camera_stream import CameraPreviewEncoder, CameraStreamConfig, iter_mjpeg
from .defect_store import DefectStore


StatisticsProvider = Callable[[], Dict[str, int]]


def create_app(
    store: DefectStore,
    statistics_provider: Optional[StatisticsProvider] = None,
    dashboard_dir: Optional[Path] = None,
    camera_store: Optional[CameraFrameStore] = None,
    camera_timeout_seconds: float = 3.0,
    camera_stream_config: Optional[CameraStreamConfig] = None,
) -> FastAPI:
    app = FastAPI(
        title="Metro Inspection Dashboard Bridge",
        version="0.1.0",
        docs_url=None,
        redoc_url=None,
        openapi_url=None,
    )
    stream_config = camera_stream_config or CameraStreamConfig()
    preview_encoder = (
        CameraPreviewEncoder(camera_store.camera_ids, stream_config)
        if camera_store is not None
        else None
    )

    @app.middleware("http")
    async def disable_dashboard_asset_cache(request, call_next):
        response = await call_next(request)
        path = request.url.path
        if path == "/" or path.endswith(".html") or path.startswith("/assets/"):
            response.headers["Cache-Control"] = "no-store"
        return response

    @app.get("/api/health")
    async def health() -> JSONResponse:
        statistics = (
            statistics_provider()
            if statistics_provider is not None
            else {"stored_events": store.count()}
        )
        return JSONResponse(
            {
                "service": "metro_dashboard_bridge",
                "status": "online",
                "defects": statistics,
                "cameras": (
                    camera_store.summary(camera_timeout_seconds)
                    if camera_store is not None
                    else {"online": 0, "total": 0}
                ),
            },
            headers={"Cache-Control": "no-store"},
        )

    @app.get("/api/defects")
    async def list_defects(
        session_id: Optional[str] = Query(
            default=None, min_length=1, max_length=200
        ),
    ) -> JSONResponse:
        selected_session_id = session_id or store.session_id
        return JSONResponse(
            {
                "session_id": selected_session_id,
                "records": store.list(session_id=selected_session_id),
            },
            headers={"Cache-Control": "no-store"},
        )

    @app.get("/api/defects/{event_id}")
    async def get_defect(
        event_id: str,
        session_id: Optional[str] = Query(
            default=None, min_length=1, max_length=200
        ),
    ) -> JSONResponse:
        record = store.get(event_id, session_id=session_id)
        if record is None:
            raise HTTPException(status_code=404, detail="defect event not found")
        return JSONResponse(record, headers={"Cache-Control": "no-store"})

    @app.get("/api/sessions")
    async def list_sessions() -> JSONResponse:
        return JSONResponse(
            {
                "active_session_id": store.session_id,
                "sessions": store.list_sessions(),
            },
            headers={"Cache-Control": "no-store"},
        )

    @app.get("/api/cameras")
    async def list_cameras() -> JSONResponse:
        cameras = (
            camera_store.list_status(camera_timeout_seconds)
            if camera_store is not None
            else []
        )
        return JSONResponse(
            {
                "cameras": cameras,
                "preview": {
                    "max_width": stream_config.max_width,
                    "max_height": stream_config.max_height,
                    "jpeg_quality": stream_config.jpeg_quality,
                    "default_fps": stream_config.default_fps,
                    "max_fps": stream_config.max_fps,
                },
            },
            headers={"Cache-Control": "no-store"},
        )

    @app.get("/api/cameras/{camera_id}/stream.mjpg")
    def camera_stream(
        camera_id: str,
        fps: float = Query(default=stream_config.default_fps, ge=1.0, le=30.0),
    ) -> StreamingResponse:
        if camera_store is None or preview_encoder is None:
            raise HTTPException(
                status_code=404, detail="camera streaming is disabled"
            )
        if not camera_store.contains(camera_id):
            raise HTTPException(status_code=404, detail="unknown camera")

        return StreamingResponse(
            iter_mjpeg(camera_store, preview_encoder, camera_id, fps),
            media_type="multipart/x-mixed-replace; boundary=frame",
            headers={
                "Cache-Control": "no-store, no-cache, must-revalidate",
                "Pragma": "no-cache",
                "X-Accel-Buffering": "no",
            },
        )

    @app.get("/api/cameras/{camera_id}/frame.jpg")
    def camera_frame(camera_id: str) -> Response:
        if camera_store is None or preview_encoder is None:
            raise HTTPException(status_code=404, detail="camera streaming is disabled")
        if not camera_store.contains(camera_id):
            raise HTTPException(status_code=404, detail="unknown camera")

        frame = camera_store.get(camera_id)
        if frame is None:
            raise HTTPException(
                status_code=503, detail="camera frame is not available"
            )
        preview = preview_encoder.encode(frame)
        if preview is None:
            raise HTTPException(
                status_code=503, detail="camera frame could not be decoded"
            )

        return Response(
            content=preview.data,
            media_type="image/jpeg",
            headers={
                "Cache-Control": "no-store, no-cache, must-revalidate",
                "Pragma": "no-cache",
                "X-Camera-Sequence": str(preview.sequence),
            },
        )

    @app.websocket("/{path:path}")
    async def reject_unknown_websocket(websocket: WebSocket, path: str) -> None:
        del path
        await websocket.close(code=1008, reason="WebSocket endpoint not available")

    if dashboard_dir is not None:
        resolved_dashboard_dir = dashboard_dir.resolve()
        if not (resolved_dashboard_dir / "index.html").is_file():
            raise FileNotFoundError(
                f"dashboard index not found: {resolved_dashboard_dir / 'index.html'}"
            )
        app.mount(
            "/",
            StaticFiles(directory=str(resolved_dashboard_dir), html=True),
            name="metro_inspection_dashboard",
        )

    return app
