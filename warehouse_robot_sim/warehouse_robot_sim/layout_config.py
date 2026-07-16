import json
from pathlib import Path
from typing import Dict, List

from ament_index_python.packages import get_package_share_directory


def package_share_path() -> Path:
    return Path(get_package_share_directory('warehouse_robot_sim'))


def available_layouts(package_share: Path = None) -> List[Dict]:
    share = package_share or package_share_path()
    layouts = []
    for path in sorted((share / 'layouts').glob('*.json')):
        layouts.append(load_layout(path.stem, share))
    return layouts


def load_layout(name: str, package_share: Path = None) -> Dict:
    if not name or not name.replace('_', '').isalnum():
        raise ValueError(f'Invalid layout name: {name}')

    share = package_share or package_share_path()
    path = share / 'layouts' / f'{name}.json'
    if not path.is_file():
        raise ValueError(f'Unknown warehouse layout: {name}')

    with path.open(encoding='utf-8') as layout_file:
        layout = json.load(layout_file)

    required = {'name', 'display_name', 'world', 'map', 'initial_poses', 'waypoints'}
    missing = required.difference(layout)
    if missing:
        raise ValueError(f'Layout {name} is missing: {", ".join(sorted(missing))}')
    return layout


def station_poses(layout: Dict) -> Dict[str, tuple]:
    return {name: tuple(pose) for name, pose in layout['waypoints'].items()}


def initial_poses(layout: Dict) -> Dict[str, tuple]:
    return {name: tuple(pose) for name, pose in layout['initial_poses'].items()}
