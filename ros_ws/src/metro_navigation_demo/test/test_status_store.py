from metro_navigation_demo.status_store import StatusStore


class FakeClock:

    def __init__(self) -> None:
        self.now = 100.0

    def __call__(self) -> float:
        return self.now


def test_ros_requires_an_external_node() -> None:
    store = StatusStore()

    initial = store.snapshot()
    assert initial['ros']['online'] is False
    assert initial['ros']['graph_checked'] is False

    store.update_ros_graph(0)
    assert store.snapshot()['ros']['online'] is False

    store.update_ros_graph(1)
    assert store.snapshot()['ros']['online'] is True


def test_graph_error_forces_ros_offline() -> None:
    store = StatusStore()
    store.update_ros_graph(3, graph_error=True)

    snapshot = store.snapshot()
    assert snapshot['ros']['online'] is False
    assert snapshot['ros']['external_node_count'] == 3
    assert snapshot['ros']['graph_error'] is True


def test_odom_freshness_expires() -> None:
    clock = FakeClock()
    store = StatusStore(odom_timeout_seconds=3.0, monotonic_clock=clock)

    store.mark_odom_received()
    assert store.snapshot()['odom']['fresh'] is True

    clock.now += 3.1
    snapshot = store.snapshot()
    assert snapshot['odom']['fresh'] is False
    assert snapshot['odom']['age_seconds'] == 3.1


def test_ros_updates_include_pose_navigation_and_safety() -> None:
    store = StatusStore()
    store.apply_ros_update({
        'type': 'odom',
        'data': {
            'pose': {'x': 1.25, 'y': -0.5, 'yaw': 0.2},
            'velocity': {'linear': 0.3, 'angular': 0.1},
            'mileage_m': 12.4,
        },
    })
    store.apply_ros_update({
        'type': 'navigation',
        'data': {'state': 'running', 'progress_percent': 42.0},
    })
    store.apply_ros_update({
        'type': 'safety',
        'data': {'estop_active': True, 'message': 'test'},
    })

    snapshot = store.snapshot()
    assert snapshot['odom']['pose']['x'] == 1.25
    assert snapshot['odom']['mileage_m'] == 12.4
    assert snapshot['navigation']['state'] == 'running'
    assert snapshot['navigation']['progress_percent'] == 42.0
    assert snapshot['safety']['estop_active'] is True
