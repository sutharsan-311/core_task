import os

from core_task_controller.function import load_perimeter, save_perimeter


def test_save_then_load_roundtrip(tmp_path):
    path = os.path.join(tmp_path, 'warehouse_perimeter.yaml')
    dock = [0.2639, 3.5269, -1.5769]
    wps = [[0.0474, -1.1304, -1.6045], [4.5532, 2.6007, -0.0163]]
    save_perimeter(path, 'warehouse', dock, wps, loops=2)

    data = load_perimeter(path)
    assert data['map'] == 'warehouse'
    assert data['frame_id'] == 'map'
    assert data['loops'] == 2
    # values are rounded to 2 decimals on write
    assert data['dock'] == [0.26, 3.53, -1.58]
    assert data['waypoints'] == [[0.05, -1.13, -1.6], [4.55, 2.6, -0.02]]


def test_written_format_is_compact(tmp_path):
    path = os.path.join(tmp_path, 'w_perimeter.yaml')
    save_perimeter(path, 'w', [0.26, 3.52, -1.57], [[0.04, -1.13, -1.6]], loops=1)
    assert open(path).read() == (
        'map: w\n'
        'frame_id: map\n'
        'loops: 1\n'
        'dock:\n'
        '  [0.26, 3.52, -1.57]\n'
        'waypoints:\n'
        '- [0.04, -1.13, -1.60]\n')


def test_no_waypoints_still_valid(tmp_path):
    path = os.path.join(tmp_path, 'empty_perimeter.yaml')
    save_perimeter(path, 'empty', [1.0, 2.0, 0.0], [])
    data = load_perimeter(path)
    assert data['dock'] == [1.0, 2.0, 0.0]
    assert data['waypoints'] is None or data['waypoints'] == []
