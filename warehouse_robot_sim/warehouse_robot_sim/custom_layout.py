import json
import math
from pathlib import Path
import re


ROWS = 8
COLS = 14
CELL_SIZE = 1.0
ORIGIN_X = -7.0
ORIGIN_Y = 4.0
MAP_ORIGIN_X = -8.0
MAP_ORIGIN_Y = -5.0
MAP_WIDTH_M = 16.0
MAP_HEIGHT_M = 10.0
MAP_RESOLUTION = 0.05
SHELF_LENGTH = 0.82
SHELF_DEPTH = 0.52
DIRECTIONS = ('north', 'east', 'south', 'west')


def make_layout(payload, output_root):
    """Validate a grid and write its layout, world, and occupancy map files."""
    display_name = str(payload.get('display_name', '')).strip()
    if not display_name or len(display_name) > 40:
        raise ValueError('Layout name must contain 1 to 40 characters.')

    layout_name = slugify(display_name)
    if not layout_name:
        raise ValueError('Layout name must include letters or numbers.')

    grid = payload.get('grid')
    validate_grid(grid)
    shelves = find_shelves(grid)
    robots = find_robots(grid)
    if len(shelves) < 2:
        raise ValueError('Add at least two shelves.')
    if 'robot1' not in robots:
        raise ValueError('Add a Robot 1 spawn point.')

    expected_robots = [f'robot{i}' for i in range(1, len(robots) + 1)]
    if sorted(robots) != expected_robots:
        raise ValueError('Robot spawn points must be numbered in order from Robot 1.')

    layout_dir = Path(output_root) / layout_name
    if layout_dir.exists():
        raise ValueError(f'A custom layout named "{display_name}" already exists.')
    layout_dir.mkdir(parents=True)

    waypoints = make_waypoints(grid, shelves)
    initial_poses = {
        robot: spawn_pose(row, col, direction)
        for robot, (row, col, direction) in sorted(robots.items())
    }

    world_path = layout_dir / f'{layout_name}.world'
    pgm_path = layout_dir / f'{layout_name}_map.pgm'
    yaml_path = layout_dir / f'{layout_name}_map.yaml'
    layout_path = layout_dir / 'layout.json'

    write_world(world_path, layout_name, shelves, waypoints)
    write_map(pgm_path, yaml_path, shelves)

    layout = {
        'name': layout_name,
        'display_name': display_name,
        'description': (
            f'Custom grid with {len(shelves)} shelves and '
            f'{len(robots)} robot spawn points.'
        ),
        'world': world_path.as_posix(),
        'map': yaml_path.as_posix(),
        'initial_poses': initial_poses,
        'waypoints': waypoints,
        'custom': True,
        'grid': grid,
    }
    layout_path.write_text(json.dumps(layout, indent=2), encoding='utf-8')
    return layout


def slugify(name):
    slug = re.sub(r'[^a-z0-9]+', '_', name.lower()).strip('_')
    return f'custom_{slug}'[:48]


def validate_grid(grid):
    if not isinstance(grid, list) or len(grid) != ROWS:
        raise ValueError(f'Layout grid must have {ROWS} rows.')

    valid_cells = {'floor', 'shelf'}
    valid_cells.update(f'shelf_{direction}' for direction in DIRECTIONS)
    for robot_number in range(1, 5):
        valid_cells.add(f'robot{robot_number}')
        valid_cells.update(
            f'robot{robot_number}_{direction}' for direction in DIRECTIONS
        )
    for row in grid:
        if not isinstance(row, list) or len(row) != COLS:
            raise ValueError(f'Each layout row must have {COLS} cells.')
        if any(cell not in valid_cells for cell in row):
            raise ValueError('Grid contains an unknown cell type.')


def find_shelves(grid):
    shelves = []
    for row in range(ROWS):
        for col in range(COLS):
            cell = grid[row][col]
            if cell == 'shelf':
                shelves.append((row, col, 'south'))
            elif cell.startswith('shelf_'):
                shelves.append((row, col, cell.split('_', 1)[1]))
    return shelves


def find_robots(grid):
    robots = {}
    for row in range(ROWS):
        for col in range(COLS):
            cell = grid[row][col]
            match = re.fullmatch(r'(robot[1-4])(?:_(north|east|south|west))?', cell)
            if not match:
                continue
            robot = match.group(1)
            direction = match.group(2)
            if robot in robots:
                raise ValueError(f'{robot.title()} can only have one spawn point.')
            robots[robot] = (row, col, direction)
    return robots


def cell_center(row, col):
    x = ORIGIN_X + (col + 0.5) * CELL_SIZE
    y = ORIGIN_Y - (row + 0.5) * CELL_SIZE
    return round(x, 3), round(y, 3)


def spawn_pose(row, col, direction=None):
    x, y = cell_center(row, col)
    if direction:
        yaw = direction_yaw(direction)
    elif abs(x) >= abs(y):
        yaw = 0.0 if x < 0 else math.pi
    else:
        yaw = -math.pi / 2 if y > 0 else math.pi / 2
    return [x, y, round(yaw, 3)]


def make_waypoints(grid, shelves):
    waypoints = {}
    used_cells = set()
    for index, (row, col, direction) in enumerate(shelves, start=1):
        row_offset, col_offset = direction_offset(direction)
        approach = (row + row_offset, col + col_offset)
        if (
            not inside_grid(*approach)
            or approach in used_cells
            or grid[approach[0]][approach[1]] != 'floor'
        ):
            raise ValueError(
                f'Shelf {index} needs an unused floor cell in front of it.'
            )

        used_cells.add(approach)
        x, y = cell_center(*approach)
        yaw = direction_yaw(opposite_direction(direction))
        waypoints[f'S{index}'] = [x, y, round(yaw, 3)]
    return waypoints


def direction_offset(direction):
    return {
        'north': (-1, 0),
        'east': (0, 1),
        'south': (1, 0),
        'west': (0, -1),
    }[direction]


def direction_yaw(direction):
    return {
        'north': math.pi / 2,
        'east': 0.0,
        'south': -math.pi / 2,
        'west': math.pi,
    }[direction]


def opposite_direction(direction):
    return {
        'north': 'south',
        'east': 'west',
        'south': 'north',
        'west': 'east',
    }[direction]


def inside_grid(row, col):
    return 0 <= row < ROWS and 0 <= col < COLS


def write_world(path, layout_name, shelves, waypoints):
    models = []
    for index, (row, col, direction) in enumerate(shelves, start=1):
        x, y = cell_center(row, col)
        size_x, size_y = shelf_size(direction)
        models.append(box_model(f'shelf_S{index}', x, y, 0.5, size_x,
                                size_y, 1.0, '0.12 0.45 0.42 1'))

    for station, (x, y, _) in waypoints.items():
        models.append(box_model(f'pad_{station}', x, y, 0.01, 0.5, 0.5,
                                0.02, '0.95 0.57 0.10 1', collision=False))

    world = f'''<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="{layout_name}_world">
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <light type="directional" name="sun">
      <pose>0 0 10 0 0 0</pose>
      <cast_shadows>true</cast_shadows>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
          <material><diffuse>0.72 0.74 0.75 1</diffuse></material>
        </visual>
      </link>
    </model>
    {box_model('north_wall', 0, 4, 1, 14, 0.2, 2, '0.18 0.20 0.22 1')}
    {box_model('south_wall', 0, -4, 1, 14, 0.2, 2, '0.18 0.20 0.22 1')}
    {box_model('west_wall', -7, 0, 1, 0.2, 8, 2, '0.18 0.20 0.22 1')}
    {box_model('east_wall', 7, 0, 1, 0.2, 8, 2, '0.18 0.20 0.22 1')}
    {''.join(models)}
  </world>
</sdf>
'''
    path.write_text(world, encoding='utf-8')


def box_model(name, x, y, z, size_x, size_y, size_z, color, collision=True):
    collision_xml = ''
    if collision:
        collision_xml = (
            '<collision name="collision"><geometry><box>'
            f'<size>{size_x} {size_y} {size_z}</size>'
            '</box></geometry></collision>'
        )
    return (
        f'<model name="{name}"><static>true</static>'
        f'<pose>{x} {y} {z} 0 0 0</pose><link name="link">'
        f'{collision_xml}<visual name="visual"><geometry><box>'
        f'<size>{size_x} {size_y} {size_z}</size></box></geometry>'
        f'<material><ambient>{color}</ambient><diffuse>{color}</diffuse>'
        '</material></visual></link></model>\n'
    )


def shelf_size(direction):
    if direction in {'north', 'south'}:
        return SHELF_LENGTH, SHELF_DEPTH
    return SHELF_DEPTH, SHELF_LENGTH


def write_map(pgm_path, yaml_path, shelves):
    width = int(MAP_WIDTH_M / MAP_RESOLUTION)
    height = int(MAP_HEIGHT_M / MAP_RESOLUTION)
    pixels = [[254 for _ in range(width)] for _ in range(height)]

    boxes = [
        (0.0, 4.0, 14.0, 0.2),
        (0.0, -4.0, 14.0, 0.2),
        (-7.0, 0.0, 0.2, 8.0),
        (7.0, 0.0, 0.2, 8.0),
    ]
    for row, col, direction in shelves:
        x, y = cell_center(row, col)
        size_x, size_y = shelf_size(direction)
        boxes.append((x, y, size_x, size_y))
    for box in boxes:
        fill_map_box(pixels, *box)

    with pgm_path.open('wb') as pgm_file:
        pgm_file.write(f'P5\n{width} {height}\n255\n'.encode('ascii'))
        for row in pixels:
            pgm_file.write(bytes(row))

    yaml_path.write_text(
        '\n'.join([
            f'image: {pgm_path.name}',
            'mode: trinary',
            f'resolution: {MAP_RESOLUTION}',
            f'origin: [{MAP_ORIGIN_X}, {MAP_ORIGIN_Y}, 0.0]',
            'negate: 0',
            'occupied_thresh: 0.65',
            'free_thresh: 0.25',
            '',
        ]),
        encoding='ascii',
    )


def fill_map_box(pixels, center_x, center_y, size_x, size_y):
    height = len(pixels)
    width = len(pixels[0])
    min_col = int((center_x - size_x / 2 - MAP_ORIGIN_X) / MAP_RESOLUTION)
    max_col = int((center_x + size_x / 2 - MAP_ORIGIN_X) / MAP_RESOLUTION)
    min_bottom = int((center_y - size_y / 2 - MAP_ORIGIN_Y) / MAP_RESOLUTION)
    max_bottom = int((center_y + size_y / 2 - MAP_ORIGIN_Y) / MAP_RESOLUTION)
    min_row = height - 1 - max_bottom
    max_row = height - 1 - min_bottom

    for row in range(max(0, min_row), min(height, max_row + 1)):
        for col in range(max(0, min_col), min(width, max_col + 1)):
            pixels[row][col] = 0
