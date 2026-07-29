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


def test_abort_from_perimeter_returns_to_dock():
    assert next_phase(Phase.PERIMETER, Event.ABORT) == Phase.RETURN_TO_DOCK


def test_abort_then_completes_the_normal_return_tail():
    # Abort re-uses the whole return-to-dock -> close -> idle tail.
    seq = [Event.SUBMIT_NAV, Event.INIT_NAV, Event.NAV_READY,
           Event.ABORT, Event.DOCK_REACHED, Event.ADVANCE, Event.NAV_CLOSED]
    assert run(seq) == Phase.IDLE


def test_abort_is_noop_outside_perimeter():
    # Guarded in the node too, but the table must not invent a transition.
    for p in (Phase.MAPPING, Phase.IDLE, Phase.RETURN_TO_DOCK, Phase.DOCKED):
        assert next_phase(p, Event.ABORT) == p


def test_find_person_happy_path_person_found_while_spinning():
    seq = [Event.SUBMIT_NAV, Event.INIT_NAV, Event.SEARCH_READY,
           Event.PERSON_FOUND, Event.PERSON_REACHED, Event.ADVANCE,
           Event.NAV_CLOSED]
    assert run(seq) == Phase.IDLE


def test_find_person_falls_back_to_patrol_after_spin():
    seq = [Event.SUBMIT_NAV, Event.INIT_NAV, Event.SEARCH_READY,
           Event.SPIN_DONE, Event.PERSON_FOUND, Event.PERSON_REACHED,
           Event.ADVANCE, Event.NAV_CLOSED]
    assert run(seq) == Phase.IDLE


def test_find_person_not_found_faults():
    # Neither the spin nor the patrol found anyone; the node enqueues ERROR.
    seq = [Event.SUBMIT_NAV, Event.INIT_NAV, Event.SEARCH_READY,
           Event.SPIN_DONE, Event.ERROR]
    assert run(seq) == Phase.FAULT


def test_abort_from_search_and_approach_returns_to_dock():
    for p in (Phase.SEARCH_SPIN, Phase.SEARCH_PATROL, Phase.APPROACHING):
        assert next_phase(p, Event.ABORT) == Phase.RETURN_TO_DOCK


def test_arrived_advances_straight_to_close_navigation():
    # No RETURN_TO_DOCK/DOCKED detour - the robot stays put near the person.
    assert next_phase(Phase.ARRIVED, Event.ADVANCE) == Phase.CLOSE_NAVIGATION
