from metro_dashboard_bridge.qt_dashboard import _health_url, _parse_arguments


def test_health_url_normalizes_trailing_slash() -> None:
    assert _health_url("http://127.0.0.1:8088/") == (
        "http://127.0.0.1:8088/api/health"
    )


def test_qt_dashboard_arguments_have_platform_defaults() -> None:
    options = _parse_arguments([])

    assert options.url == "http://127.0.0.1:8088"
    assert options.startup_timeout == 30.0
