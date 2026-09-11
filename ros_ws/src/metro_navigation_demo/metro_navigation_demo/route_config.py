import json
from pathlib import Path
from typing import Any, Dict, List


class RouteCatalog:
    """Validated route and junction definitions shared by ROS and HTTP."""

    def __init__(self, config_path: Path) -> None:
        self.path = config_path.resolve()
        with self.path.open(encoding='utf-8') as stream:
            config = json.load(stream)
        self._config = self._validate(config)

    @staticmethod
    def _validate_waypoints(name: str, value: object) -> List[Dict[str, float]]:
        if not isinstance(value, list) or not value:
            raise ValueError(f'{name} requires at least one waypoint')
        result = []
        for index, waypoint in enumerate(value):
            if not isinstance(waypoint, dict):
                raise ValueError(f'{name} waypoint {index} must be an object')
            converted = {}
            for field in ('x', 'y', 'yaw'):
                number = waypoint.get(field)
                if isinstance(number, bool) or not isinstance(number, (int, float)):
                    raise ValueError(f'{name} waypoint {index} requires numeric {field}')
                converted[field] = float(number)
            result.append(converted)
        return result

    @classmethod
    def _validate(cls, config: object) -> Dict[str, Any]:
        if not isinstance(config, dict):
            raise ValueError('route config must be an object')
        frame_id = config.get('frame_id')
        routes = config.get('routes')
        if not isinstance(frame_id, str) or not frame_id:
            raise ValueError('route config requires a non-empty frame_id')
        if not isinstance(routes, dict) or not routes:
            raise ValueError('route config requires at least one route')

        validated_routes = {}
        for route_name, route in routes.items():
            if not isinstance(route_name, str) or not isinstance(route, dict):
                raise ValueError('routes must be named objects')
            validated_routes[route_name] = {
                'label': str(route.get('label', route_name)),
                'waypoints': cls._validate_waypoints(
                    f'route {route_name!r}', route.get('waypoints')
                ),
            }

        destinations = config.get('destinations')
        if destinations is None:
            destinations = {
                route_name: {
                    'label': route['label'],
                    'route': route_name,
                }
                for route_name, route in validated_routes.items()
            }
        if not isinstance(destinations, dict) or not destinations:
            raise ValueError('destinations must be a non-empty object')
        validated_destinations = {}
        for key, destination in destinations.items():
            if not isinstance(destination, dict):
                raise ValueError(f'destination {key!r} must be an object')
            route_name = destination.get('route')
            if route_name not in validated_routes:
                raise ValueError(f'destination {key!r} references unknown route')
            validated_destinations[key] = {
                'label': str(destination.get('label', key)),
                'route': route_name,
            }

        junction = config.get('junction')
        if not isinstance(junction, dict):
            raise ValueError('route config requires a junction object')
        common = cls._validate_waypoints('junction common route', junction.get('common_waypoints'))
        choices = junction.get('choices')
        if not isinstance(choices, dict) or not choices:
            raise ValueError('junction requires at least one choice')
        validated_choices = {}
        for key, choice in choices.items():
            if not isinstance(choice, dict):
                raise ValueError(f'junction choice {key!r} must be an object')
            route_name = choice.get('route')
            start_index = choice.get('start_index')
            if route_name not in validated_routes:
                raise ValueError(f'junction choice {key!r} references unknown route')
            if not isinstance(start_index, int) or isinstance(start_index, bool):
                raise ValueError(f'junction choice {key!r} requires integer start_index')
            route_waypoints = validated_routes[route_name]['waypoints']
            if start_index < 0 or start_index >= len(route_waypoints):
                raise ValueError(f'junction choice {key!r} start_index is out of range')
            validated_choices[key] = {
                'label': str(choice.get('label', validated_routes[route_name]['label'])),
                'route': route_name,
                'start_index': start_index,
            }

        return {
            'world': str(config.get('world', 'unknown')),
            'frame_id': frame_id,
            'routes': validated_routes,
            'destinations': validated_destinations,
            'junction': {
                'label': str(junction.get('label', '前方岔路口')),
                'common_waypoints': common,
                'choices': validated_choices,
            },
        }

    @property
    def frame_id(self) -> str:
        return self._config['frame_id']

    def destination_route(self, destination: str) -> tuple[str, str, List[Dict[str, float]]]:
        item = self._config['destinations'][destination]
        route = self._config['routes'][item['route']]
        return item['route'], item['label'], list(route['waypoints'])

    def junction_common_route(self) -> tuple[str, List[Dict[str, float]]]:
        junction = self._config['junction']
        return junction['label'], list(junction['common_waypoints'])

    def junction_choice_route(self, choice: str) -> tuple[str, str, List[Dict[str, float]]]:
        item = self._config['junction']['choices'][choice]
        route = self._config['routes'][item['route']]
        return item['route'], item['label'], list(route['waypoints'][item['start_index']:])

    def public_options(self) -> Dict[str, object]:
        return {
            'world': self._config['world'],
            'frame_id': self.frame_id,
            'modes': [
                {'id': 'junction', 'label': '到岔口询问'},
                {'id': 'destination', 'label': '按最终目的地'},
            ],
            'destinations': [
                {'id': key, 'label': value['label'], 'route': value['route']}
                for key, value in self._config['destinations'].items()
            ],
            'junction': {
                'label': self._config['junction']['label'],
                'choices': [
                    {'id': key, 'label': value['label'], 'route': value['route']}
                    for key, value in self._config['junction']['choices'].items()
                ],
            },
        }

    def has_destination(self, destination: str) -> bool:
        return destination in self._config['destinations']

    def has_junction_choice(self, choice: str) -> bool:
        return choice in self._config['junction']['choices']
