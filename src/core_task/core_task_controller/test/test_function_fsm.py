from core_task_controller.function import Event, Phase, next_phase


def run(seq, start=Phase.IDLE):
    p = start
    for ev in seq:
        p = next_phase(p, ev)
    return p


def test_mapping_happy_path():
    seq = [Event.SUBMIT_MAPPING, Event.INIT_MAPPING, Event.SLAM_READY,
           Event.OPERATOR_DONE, Event.SAVE_OK, Event.ADVANCE,
           Event.SLAM_CLOSED, Event.OPERATOR_DONE, Event.ADVANCE]
    assert run(seq) == Phase.IDLE


def test_navigation_happy_path():
    seq = [Event.SUBMIT_NAV, Event.INIT_NAV, Event.NAV_READY,
           Event.LOOPS_DONE, Event.ADVANCE, Event.DOCK_REACHED,
           Event.ADVANCE, Event.NAV_CLOSED]
    assert run(seq) == Phase.IDLE


def test_collect_goals_happy_path():
    seq = [Event.SUBMIT_GOALS, Event.INIT_GOALS,
           Event.OPERATOR_DONE, Event.ADVANCE]
    assert run(seq) == Phase.IDLE


def test_operator_done_branches_by_phase():
    assert next_phase(Phase.MAPPING, Event.OPERATOR_DONE) == Phase.SAVING
    assert (next_phase(Phase.GOALPOINT_COLLECTION, Event.OPERATOR_DONE)
            == Phase.GOAL_POINTS_SAVED)


def test_error_always_faults():
    for p in (Phase.MAPPING, Phase.PERIMETER, Phase.SAVING):
        assert next_phase(p, Event.ERROR) == Phase.FAULT


def test_invalid_mission_faults():
    assert next_phase(Phase.INITIALIZATION, Event.SUBMIT_INVALID) == Phase.FAULT


def test_new_mission_clears_fault():
    assert next_phase(Phase.FAULT, Event.SUBMIT_NAV) == Phase.INITIALIZATION


def test_unmapped_event_is_noop():
    assert next_phase(Phase.MAPPING, Event.SLAM_READY) == Phase.MAPPING
