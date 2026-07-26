from core_task_controller.cmd_vel_limiter import clamp


def test_clamp():
    assert clamp(5.0, 0.26) == 0.26      # over -> capped
    assert clamp(-5.0, 0.26) == -0.26    # under -> capped
    assert clamp(0.1, 0.26) == 0.1       # within -> untouched
    assert clamp(0.0, 1.82) == 0.0
