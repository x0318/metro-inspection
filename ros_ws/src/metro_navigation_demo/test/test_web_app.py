from pathlib import Path

from fastapi.testclient import TestClient
import pytest

from metro_navigation_demo.defect_store import DefectStore
from metro_navigation_demo.route_config import RouteCatalog
from metro_navigation_demo.status_store import StatusStore
from metro_navigation_demo.web_app import create_app


PROJECT_DIR = Path(__file__).resolve().parents[1]


@pytest.fixture(autouse=True)
def legacy_camera_profile(monkeypatch):
    monkeypatch.setenv('PLATFORM_CAMERA_PROFILE', 'subway_v2')


class RecordingGateway:

    def __init__(self) -> None:
        self.commands = []

    def send(self, command: dict) -> None:
        self.commands.append(command)


def make_client(tmp_path: Path) -> tuple[TestClient, RecordingGateway]:
    gateway = RecordingGateway()
    app = create_app(
        StatusStore(),
        PROJECT_DIR / 'web' / 'inspection_dashboard',
        gateway,
        RouteCatalog(PROJECT_DIR / 'config' / 'route_choice_training_routes.json'),
        DefectStore(),
        tmp_path,
    )
    return TestClient(app), gateway


def test_mutations_require_session_token(tmp_path: Path) -> None:
    client, gateway = make_client(tmp_path)
    assert client.post('/api/navigation/pause').status_code == 403

    token = client.get('/api/session').json()['token']
    response = client.post(
        '/api/navigation/start',
        headers={'X-Session-Token': token},
        json={'mode': 'destination', 'destination': 'branch_end'},
    )

    assert response.status_code == 200
    assert gateway.commands[-1] == {
        'command': 'start',
        'mode': 'destination',
        'destination': 'branch_end',
    }


def test_unknown_destination_is_rejected(tmp_path: Path) -> None:
    client, gateway = make_client(tmp_path)
    token = client.get('/api/session').json()['token']
    response = client.post(
        '/api/navigation/start',
        headers={'X-Session-Token': token},
        json={'mode': 'destination', 'destination': 'missing'},
    )

    assert response.status_code == 422
    assert gateway.commands == []


def test_camera_selection_is_whitelisted(tmp_path: Path) -> None:
    client, gateway = make_client(tmp_path)
    token = client.get('/api/session').json()['token']
    response = client.post(
        '/api/camera/select',
        headers={'X-Session-Token': token},
        json={'camera_name': 'pitch'},
    )
    assert response.status_code == 200
    assert gateway.commands[-1] == {
        'command': 'select_camera',
        'camera_name': 'pitch',
    }

    response = client.post(
        '/api/camera/select',
        headers={'X-Session-Token': token},
        json={'camera_name': '/arbitrary/topic'},
    )
    assert response.status_code == 422


def test_all_five_camera_endpoints_are_independent(tmp_path: Path) -> None:
    client, _ = make_client(tmp_path)
    camera_names = ('xj1', 'xj2', 'xj3', 'xj4', 'pitch')

    for index, camera_name in enumerate(camera_names):
        frame = b'jpeg-' + camera_name.encode('ascii') + bytes([index])
        (tmp_path / f'{camera_name}.jpg').write_bytes(frame)
        response = client.get(f'/api/camera/{camera_name}.jpg')
        assert response.status_code == 200
        assert response.content == frame


def test_training_camera_profile_matches_existing_robot(tmp_path, monkeypatch):
    monkeypatch.setenv('PLATFORM_CAMERA_PROFILE', 'route_training')
    client, gateway = make_client(tmp_path)
    cameras = client.get('/api/cameras').json()['cameras']
    assert [camera['id'] for camera in cameras] == ['front', 'left', 'right', 'ground']
    assert cameras[0]['topic'] == '/zed2i_depth/image_raw/compressed'
    token = client.get('/api/session').json()['token']
    for camera in cameras:
        name = camera['id']
        (tmp_path / f'{name}.jpg').write_bytes(b'frame-' + name.encode())
        assert client.get(f'/api/camera/{name}.jpg').content == b'frame-' + name.encode()
        assert client.post('/api/camera/select', headers={'X-Session-Token': token},
                           json={'camera_name': name}).status_code == 200
        assert gateway.commands[-1]['camera_name'] == name
    assert client.get('/api/camera/xj1.jpg').status_code == 404
    assert client.post('/api/camera/select', headers={'X-Session-Token': token},
                       json={'camera_name': 'xj1'}).status_code == 422

    assert client.get('/api/camera/odin1.jpg').status_code == 404


def test_installed_symlink_assets_can_be_served(tmp_path):
    source = tmp_path / 'source'
    installed = tmp_path / 'installed'
    source.mkdir()
    installed.mkdir()
    (source / 'index.html').write_text('<html>navigation</html>')
    (source / 'app.js').write_text('const ready = true;')
    for name in ('index.html', 'app.js'):
        (installed / name).symlink_to(source / name)
    app = create_app(StatusStore(), installed, RecordingGateway(),
                     RouteCatalog(PROJECT_DIR / 'config/route_choice_training_routes.json'),
                     DefectStore(), tmp_path)
    client = TestClient(app)
    assert client.get('/').status_code == 200
    assert client.get('/app.js').text == 'const ready = true;'
    assert client.get('/../outside.txt').status_code == 404


def test_real_defect_ingest_and_list(tmp_path: Path) -> None:
    client, _ = make_client(tmp_path)
    token = client.get('/api/session').json()['token']
    response = client.post(
        '/api/defects',
        headers={'X-Session-Token': token},
        json={
            'type': '裂缝',
            'level': 'II级 (中度)',
            'mileage': 'K12+220.5',
            'ring_number': 18,
            'clock_position': 2.0,
            'position': {'x': 1.2, 'y': 0.4, 'z': 1.6},
            'confidence': 0.93,
            'camera_name': 'xj1',
            'bbox': {
                'x': 100, 'y': 80, 'width': 120, 'height': 160,
                'image_width': 1440, 'image_height': 1080,
            },
        },
    )

    assert response.status_code == 201
    records = client.get('/api/defects').json()['records']
    assert len(records) == 1
    assert records[0]['type'] == '裂缝'
    assert records[0]['confidence'] == 0.93
