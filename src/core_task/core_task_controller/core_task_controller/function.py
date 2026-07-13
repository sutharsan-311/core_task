"""Pure, ROS-free logic for Operation_controller: mission validation, the FSM
transition table, perimeter file I/O, and pose math. No rclpy imports here so
it stays unit-testable without a ROS graph."""
import math
from enum import Enum, auto

import yaml

VALID_MODES = ('mapping', 'navigation', 'collect_goals')


# --------------------------------------------------------------------------
# Mission validation
# --------------------------------------------------------------------------
def validate_mission(mission):
    """Return (ok, reason). A mission is {mode, map_name, loops?}."""
    if not isinstance(mission, dict):
        return False, 'mission must be a JSON object'
    mode = mission.get('mode')
    if mode not in VALID_MODES:
        return False, "mode must be 'mapping' or 'navigation'"
    if not mission.get('map_name'):
        return False, 'map_name is required'
    if mode == 'navigation':
        loops = mission.get('loops', 1)
        if not isinstance(loops, int) or loops < 1:
            return False, 'loops must be a positive integer'
    return True, 'ok'


# --------------------------------------------------------------------------
# Pose math
# --------------------------------------------------------------------------
def yaw_to_quaternion(yaw):
    """Yaw (rad) -> (x, y, z, w) for a flat-ground robot."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def quaternion_to_yaw(x, y, z, w):
    """Full-quaternion -> yaw (rad)."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


# --------------------------------------------------------------------------
# Perimeter file I/O
# --------------------------------------------------------------------------
def save_perimeter(path, map_name, dock, waypoints, loops=1):
    data = {
        'map': map_name,
        'frame_id': 'map',
        'loops': loops,
        'dock': dock,
        'waypoints': waypoints,
    }
    with open(path, 'w') as f:
        yaml.safe_dump(data, f, sort_keys=False)


def load_perimeter(path):
    with open(path) as f:
        return yaml.safe_load(f)


# --------------------------------------------------------------------------
# Finite-state machine
# --------------------------------------------------------------------------
class Phase(Enum):
    IDLE = auto()
    INITIALIZATION = auto()
    START_MAPPING = auto()
    MAPPING = auto()
    SAVING = auto()
    MAP_SAVED_SUCCESSFULLY = auto()
    CLOSE_MAPPING = auto()
    GOALPOINT_COLLECTION = auto()
    GOAL_POINTS_SAVED = auto()
    START_NAVIGATION = auto()
    PERIMETER = auto()
    PERIMETER_COMPLETED = auto()
    RETURN_TO_DOCK = auto()
    DOCKED = auto()
    CLOSE_NAVIGATION = auto()
    FAULT = auto()


class Event(Enum):
    SUBMIT_MAPPING = auto()
    SUBMIT_NAV = auto()
    SUBMIT_GOALS = auto()
    INIT_MAPPING = auto()
    INIT_NAV = auto()
    INIT_GOALS = auto()
    SUBMIT_INVALID = auto()
    SLAM_READY = auto()
    OPERATOR_DONE = auto()
    SAVE_OK = auto()
    SLAM_CLOSED = auto()
    PERIMETER_SAVED = auto()
    NAV_READY = auto()
    LOOPS_DONE = auto()
    DOCK_REACHED = auto()
    NAV_CLOSED = auto()
    ADVANCE = auto()
    ERROR = auto()


_TRANSITIONS = {
    (Phase.IDLE, Event.SUBMIT_MAPPING): Phase.INITIALIZATION,
    (Phase.IDLE, Event.SUBMIT_NAV): Phase.INITIALIZATION,
    (Phase.IDLE, Event.SUBMIT_GOALS): Phase.INITIALIZATION,
    (Phase.FAULT, Event.SUBMIT_MAPPING): Phase.INITIALIZATION,
    (Phase.FAULT, Event.SUBMIT_NAV): Phase.INITIALIZATION,
    (Phase.FAULT, Event.SUBMIT_GOALS): Phase.INITIALIZATION,
    (Phase.INITIALIZATION, Event.INIT_MAPPING): Phase.START_MAPPING,
    (Phase.INITIALIZATION, Event.INIT_NAV): Phase.START_NAVIGATION,
    (Phase.INITIALIZATION, Event.INIT_GOALS): Phase.GOALPOINT_COLLECTION,
    (Phase.INITIALIZATION, Event.SUBMIT_INVALID): Phase.FAULT,
    (Phase.START_MAPPING, Event.SLAM_READY): Phase.MAPPING,
    (Phase.MAPPING, Event.OPERATOR_DONE): Phase.SAVING,
    (Phase.SAVING, Event.SAVE_OK): Phase.MAP_SAVED_SUCCESSFULLY,
    (Phase.MAP_SAVED_SUCCESSFULLY, Event.ADVANCE): Phase.CLOSE_MAPPING,
    (Phase.CLOSE_MAPPING, Event.SLAM_CLOSED): Phase.GOALPOINT_COLLECTION,
    (Phase.GOALPOINT_COLLECTION, Event.OPERATOR_DONE): Phase.GOAL_POINTS_SAVED,
    (Phase.GOAL_POINTS_SAVED, Event.ADVANCE): Phase.IDLE,
    (Phase.START_NAVIGATION, Event.NAV_READY): Phase.PERIMETER,
    (Phase.PERIMETER, Event.LOOPS_DONE): Phase.PERIMETER_COMPLETED,
    (Phase.PERIMETER_COMPLETED, Event.ADVANCE): Phase.RETURN_TO_DOCK,
    (Phase.RETURN_TO_DOCK, Event.DOCK_REACHED): Phase.DOCKED,
    (Phase.DOCKED, Event.ADVANCE): Phase.CLOSE_NAVIGATION,
    (Phase.CLOSE_NAVIGATION, Event.NAV_CLOSED): Phase.IDLE,
}


def next_phase(phase, event):
    """Pure transition. ERROR -> FAULT from anywhere; unmapped pair is a no-op."""
    if event == Event.ERROR:
        return Phase.FAULT
    return _TRANSITIONS.get((phase, event), phase)
