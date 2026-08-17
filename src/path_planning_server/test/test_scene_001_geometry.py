from pathlib import Path

import pytest
import yaml


SCENE_FILE = (
    Path(__file__).parents[1] / 'config' / 'scenes' / 'scene_001.yaml'
)


def test_scene_001_table_and_open_trapezoid_box_geometry():
    scene = yaml.safe_load(SCENE_FILE.read_text(encoding='utf-8'))
    table = scene['table']
    box = scene['open_trapezoid_box']

    assert table['size_x'] == pytest.approx(1.0)
    assert table['size_y'] == pytest.approx(1.2)
    assert table['center_y'] == pytest.approx(0.0)

    near_table_edge_x = table['center_x'] - table['size_x'] / 2.0
    assert near_table_edge_x == pytest.approx(-0.07)

    assert box['type'] == 'open_trapezoid_box'
    assert box['long_axis'] == 'y'
    assert box['bottom_length'] == pytest.approx(0.3)
    assert box['bottom_width'] == pytest.approx(0.2)
    assert box['top_length'] == pytest.approx(0.4)
    assert box['top_width'] == pytest.approx(0.3)
    assert box['height'] == pytest.approx(0.2)
    assert box['thickness'] == pytest.approx(0.005)
    assert box['bottom_z'] == pytest.approx(table['top_z'])
    assert box['center_y'] == pytest.approx(table['center_y'])

    assert box['center_x'] == pytest.approx(0.22)
    assert box['center_x'] - near_table_edge_x == pytest.approx(0.29)


def test_scene_001_shelf_geometry():
    scene = yaml.safe_load(SCENE_FILE.read_text(encoding='utf-8'))
    table = scene['table']
    shelf = scene['shelf']

    assert shelf['type'] == 'shelf'
    assert shelf['long_axis'] == 'y'
    assert shelf['length'] == pytest.approx(0.5)
    assert shelf['width'] == pytest.approx(0.265)
    assert shelf['height'] == pytest.approx(0.44)

    assert shelf['board_length'] == pytest.approx(0.5)
    assert shelf['board_width'] == pytest.approx(0.26)
    assert shelf['board_thickness'] == pytest.approx(0.015)
    assert shelf['board_center_heights'] == pytest.approx(
        [0.035, 0.22]
    )
    assert shelf['post_diameter'] == pytest.approx(0.015)
    assert shelf['bottom_z'] == pytest.approx(table['top_z'])
    assert shelf['center_y'] == pytest.approx(table['center_y'])

    near_table_edge_x = table['center_x'] - table['size_x'] / 2.0
    assert shelf['center_x'] == pytest.approx(0.525)
    assert shelf['center_x'] - near_table_edge_x == pytest.approx(0.595)

    shelf_min_x = shelf['center_x'] - shelf['width'] / 2.0
    shelf_max_x = shelf['center_x'] + shelf['width'] / 2.0
    shelf_min_y = shelf['center_y'] - shelf['length'] / 2.0
    shelf_max_y = shelf['center_y'] + shelf['length'] / 2.0
    assert shelf_min_x >= table['center_x'] - table['size_x'] / 2.0
    assert shelf_max_x <= table['center_x'] + table['size_x'] / 2.0
    assert shelf_min_y >= table['center_y'] - table['size_y'] / 2.0
    assert shelf_max_y <= table['center_y'] + table['size_y'] / 2.0
