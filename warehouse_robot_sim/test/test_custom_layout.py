import json

import pytest

from warehouse_robot_sim.custom_layout import COLS, ROWS, make_layout


def example_grid():
    grid = [['floor' for _ in range(COLS)] for _ in range(ROWS)]
    grid[1][3] = 'shelf_south'
    grid[6][10] = 'shelf_north'
    grid[3][0] = 'robot1_east'
    grid[4][0] = 'robot2_east'
    return grid


def test_make_layout_writes_all_files(tmp_path):
    layout = make_layout(
        {'display_name': 'Test Warehouse', 'grid': example_grid()},
        tmp_path,
    )

    layout_dir = tmp_path / 'custom_test_warehouse'
    assert layout['name'] == 'custom_test_warehouse'
    assert len(layout['waypoints']) == 2
    assert len(layout['initial_poses']) == 2
    assert (layout_dir / 'custom_test_warehouse.world').is_file()
    assert (layout_dir / 'custom_test_warehouse_map.pgm').is_file()
    assert (layout_dir / 'custom_test_warehouse_map.yaml').is_file()
    assert json.loads((layout_dir / 'layout.json').read_text())['custom'] is True


def test_layout_requires_robot_one(tmp_path):
    grid = example_grid()
    grid[3][0] = 'floor'

    with pytest.raises(ValueError, match='Robot 1'):
        make_layout({'display_name': 'No Robot', 'grid': grid}, tmp_path)


def test_shelf_orientation_controls_checkpoint(tmp_path):
    layout = make_layout(
        {'display_name': 'Rotated Shelves', 'grid': example_grid()},
        tmp_path,
    )

    assert layout['waypoints']['S1'][:2] == [-3.5, 1.5]
    assert layout['waypoints']['S2'][:2] == [3.5, -1.5]
    assert layout['initial_poses']['robot1'][2] == 0.0


def test_shelf_front_must_be_open(tmp_path):
    grid = example_grid()
    grid[2][3] = 'robot3_north'

    with pytest.raises(ValueError, match='in front'):
        make_layout({'display_name': 'Blocked Shelf', 'grid': grid}, tmp_path)
