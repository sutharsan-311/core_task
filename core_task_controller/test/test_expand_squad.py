from core_task_controller.function import expand_squad


def test_expands_to_two_navigation_missions():
    squad = {'mode': 'squad_navigation', 'map_name': 'warehouse', 'loops': 2}
    out = expand_squad(squad, n=2)
    assert len(out) == 2
    assert out[0] == {'mode': 'navigation', 'map_name': 'warehouse',
                      'loops': 2, 'robots': 2, 'robot_index': 0}
    assert out[1] == {'mode': 'navigation', 'map_name': 'warehouse',
                      'loops': 2, 'robots': 2, 'robot_index': 1}


def test_loops_defaults_to_one():
    out = expand_squad({'mode': 'squad_navigation', 'map_name': 'w'}, n=2)
    assert all(m['loops'] == 1 for m in out)
