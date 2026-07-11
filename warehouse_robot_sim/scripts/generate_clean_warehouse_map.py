#!/usr/bin/env python3
"""Generate a clean occupancy-grid map matching worlds/warehouse.world.

This map is intentionally generated from known Gazebo world geometry for stable
Nav2 testing. It does not replace the SLAM map; keep the SLAM map as proof of
mapping and use this one for repeatable navigation experiments.
"""

from pathlib import Path

RESOLUTION = 0.05
ORIGIN_X = -8.0
ORIGIN_Y = -5.0
WIDTH_M = 16.0
HEIGHT_M = 10.0
WIDTH = int(WIDTH_M / RESOLUTION)
HEIGHT = int(HEIGHT_M / RESOLUTION)

FREE = 254
OCCUPIED = 0

# name, center_x, center_y, size_x, size_y
BOXES = [
    ("north_wall", 0.0, 4.0, 14.0, 0.2),
    ("south_wall", 0.0, -4.0, 14.0, 0.2),
    ("west_wall", -7.0, 0.0, 0.2, 8.0),
    ("east_wall", 7.0, 0.0, 0.2, 8.0),
    ("shelf_A1", -4.0, 2.0, 1.5, 0.5),
    ("shelf_A2", 0.0, 2.0, 1.5, 0.5),
    ("shelf_A3", 4.0, 2.0, 1.5, 0.5),
    ("shelf_B1", -4.0, -2.0, 1.5, 0.5),
    ("shelf_B2", 0.0, -2.0, 1.5, 0.5),
    ("shelf_B3", 4.0, -2.0, 1.5, 0.5),
]


def world_to_pixel(x: float, y: float):
    col = int((x - ORIGIN_X) / RESOLUTION)
    row_from_bottom = int((y - ORIGIN_Y) / RESOLUTION)
    row = HEIGHT - 1 - row_from_bottom
    return col, row


def fill_box(grid, center_x: float, center_y: float, size_x: float, size_y: float):
    min_x = center_x - size_x / 2.0
    max_x = center_x + size_x / 2.0
    min_y = center_y - size_y / 2.0
    max_y = center_y + size_y / 2.0

    min_col, max_row = world_to_pixel(min_x, min_y)
    max_col, min_row = world_to_pixel(max_x, max_y)

    min_col = max(0, min(WIDTH - 1, min_col))
    max_col = max(0, min(WIDTH - 1, max_col))
    min_row = max(0, min(HEIGHT - 1, min_row))
    max_row = max(0, min(HEIGHT - 1, max_row))

    for row in range(min_row, max_row + 1):
        for col in range(min_col, max_col + 1):
            grid[row][col] = OCCUPIED


def write_pgm(path: Path, grid):
    with path.open("wb") as pgm:
        pgm.write(f"P5\n{WIDTH} {HEIGHT}\n255\n".encode("ascii"))
        for row in grid:
            pgm.write(bytes(row))


def write_yaml(path: Path, image_name: str):
    path.write_text(
        "\n".join(
            [
                f"image: {image_name}",
                "mode: trinary",
                f"resolution: {RESOLUTION}",
                f"origin: [{ORIGIN_X}, {ORIGIN_Y}, 0.0]",
                "negate: 0",
                "occupied_thresh: 0.65",
                "free_thresh: 0.25",
                "",
            ]
        ),
        encoding="ascii",
    )


def main():
    package_dir = Path(__file__).resolve().parents[1]
    maps_dir = package_dir / "maps"
    maps_dir.mkdir(parents=True, exist_ok=True)

    grid = [[FREE for _ in range(WIDTH)] for _ in range(HEIGHT)]
    for _, x, y, sx, sy in BOXES:
        fill_box(grid, x, y, sx, sy)

    pgm_path = maps_dir / "clean_warehouse_map.pgm"
    yaml_path = maps_dir / "clean_warehouse_map.yaml"
    write_pgm(pgm_path, grid)
    write_yaml(yaml_path, pgm_path.name)

    print(f"Wrote {pgm_path}")
    print(f"Wrote {yaml_path}")


if __name__ == "__main__":
    main()
