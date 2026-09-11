import json
from pathlib import Path

import pytest

from metro_navigation_demo.route_config import RouteCatalog


PROJECT_DIR = Path(__file__).resolve().parents[1]


def test_training_route_supports_both_selection_modes() -> None:
    catalog = RouteCatalog(PROJECT_DIR / 'config' / 'route_choice_training_routes.json')
    options = catalog.public_options()

    assert {item['id'] for item in options['modes']} == {'junction', 'destination'}
    assert {item['id'] for item in options['destinations']} == {'main_end', 'branch_end'}
    assert {item['id'] for item in options['junction']['choices']} == {'straight', 'branch'}

    route_name, _, branch_waypoints = catalog.junction_choice_route('branch')
    assert route_name == 'branch'
    assert branch_waypoints[0]['x'] == pytest.approx(-1.76)


def test_invalid_junction_index_is_rejected(tmp_path: Path) -> None:
    config = json.loads(
        (PROJECT_DIR / 'config' / 'route_choice_training_routes.json').read_text()
    )
    config['junction']['choices']['branch']['start_index'] = 999
    config_path = tmp_path / 'bad_routes.json'
    config_path.write_text(json.dumps(config), encoding='utf-8')

    with pytest.raises(ValueError, match='out of range'):
        RouteCatalog(config_path)
