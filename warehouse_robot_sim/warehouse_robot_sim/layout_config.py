import json
import os
from pathlib import Path
from typing import Dict, List

from ament_index_python.packages import get_package_share_directory


def package_share_path() -> Path:
    return Path(get_package_share_directory('warehouse_robot_sim'))


def custom_layout_root(package_share: Path = None) -> Path:
    value = os.environ.get('WAREHOUSE_CUSTOM_LAYOUT_DIR', '').strip()
    if value:
        return Path(value)

    share = package_share or package_share_path()
    for parent in [share] + list(share.parents):
        if (parent / 'install' / 'setup.bash').exists():
            return parent / 'custom_layouts'
    return None


def available_layouts(
    package_share: Path = None,
    custom_root: Path = None,
) -> List[Dict]:
    share = package_share or package_share_path()
    layouts = []
    for path in sorted((share / 'layouts').glob('*.json')):
        layouts.append(load_layout(path.stem, share))
    root = custom_root or custom_layout_root(share)
    if root and root.exists():
        for path in sorted(root.glob('*/layout.json')):
            layouts.append(read_layout(path))
    return layouts


def load_layout(name: str, package_share: Path = None) -> Dict:
    if not name or not name.replace('_', '').isalnum():
        raise ValueError(f'Invalid layout name: {name}')

    share = package_share or package_share_path()
    path = share / 'layouts' / f'{name}.json'
    root = custom_layout_root(share)
    custom_path = root / name / 'layout.json' if root else None
    if custom_path and custom_path.is_file():
        path = custom_path
    if not path.is_file():
        raise ValueError(f'Unknown warehouse layout: {name}')

    return read_layout(path)


def read_layout(path: Path) -> Dict:
    with path.open(encoding='utf-8') as layout_file:
        layout = json.load(layout_file)

    required = {'name', 'display_name', 'world', 'map', 'initial_poses', 'waypoints'}
    missing = required.difference(layout)
    if missing:
        name = layout.get('name', path.stem)
        raise ValueError(f'Layout {name} is missing: {", ".join(sorted(missing))}')
    return layout


def station_poses(layout: Dict) -> Dict[str, tuple]:
    return {name: tuple(pose) for name, pose in layout['waypoints'].items()}


def initial_poses(layout: Dict) -> Dict[str, tuple]:
    return {name: tuple(pose) for name, pose in layout['initial_poses'].items()}
