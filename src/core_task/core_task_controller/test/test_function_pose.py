import math

from core_task_controller.function import quaternion_to_yaw, yaw_to_quaternion


def test_zero_yaw():
    x, y, z, w = yaw_to_quaternion(0.0)
    assert (x, y, z) == (0.0, 0.0, 0.0) and abs(w - 1.0) < 1e-9


def test_roundtrip():
    for yaw in (-2.0, -0.5, 0.0, 1.2, 3.0):
        x, y, z, w = yaw_to_quaternion(yaw)
        assert abs(quaternion_to_yaw(x, y, z, w) - yaw) < 1e-6


def test_half_pi():
    _, _, z, w = yaw_to_quaternion(math.pi / 2)
    assert abs(z - math.sqrt(0.5)) < 1e-9 and abs(w - math.sqrt(0.5)) < 1e-9
