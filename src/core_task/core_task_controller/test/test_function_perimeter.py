import os

from core_task_controller.function import load_perimeter, save_perimeter


def test_save_then_load(tmp_path):
    path = os.path.join(tmp_path, 'warehouse_perimeter.yaml')
    dock = {'x': 6.56, 'y': 2.18, 'yaw': 3.14}
    wps = [{'x': 1.0, 'y': 2.0, 'yaw': 0.0}, {'x': 3.0, 'y': 2.0, 'yaw': 1.57}]
    save_perimeter(path, 'warehouse', dock, wps, loops=2)

    data = load_perimeter(path)
    assert data['map'] == 'warehouse'
    assert data['frame_id'] == 'map'
    assert data['loops'] == 2
    assert data['dock'] == dock
    assert data['waypoints'] == wps
