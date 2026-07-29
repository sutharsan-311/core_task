"""Pure, ROS-free logic for Operation_controller: mission validation, the FSM
transition table, perimeter file I/O, and pose math. No rclpy imports here so
it stays unit-testable without a ROS graph."""
import math
from enum import Enum, auto

import yaml

VALID_MODES = ('mapping', 'navigation', 'collect_goals', 'squad_navigation',
               'find_person')


def split_waypoints(waypoints, n, i):
    """Return the i-th of n contiguous chunks of `waypoints` (0-indexed).

    Chunks are contiguous and in order; when the count does not divide
    evenly the front chunks each absorb one extra, so for n=2 robot 0 takes
    the larger front half. n <= 1 returns the whole list unchanged.
    """
    wps = list(waypoints)
    if n <= 1:
        return wps
    base, extra = divmod(len(wps), n)
    start = i * base + min(i, extra)
    size = base + (1 if i < extra else 0)
    return wps[start:start + size]


# --------------------------------------------------------------------------
# Mission validation
# --------------------------------------------------------------------------
def validate_mission(mission):
    """Return (ok, reason). A mission is {mode, map_name, loops?, robots?,
    robot_index?}."""
    if not isinstance(mission, dict):
        return False, 'mission must be a JSON object'
    mode = mission.get('mode')
    if mode not in VALID_MODES:
        return False, 'mode must be one of %s' % (VALID_MODES,)
    if not mission.get('map_name'):
        return False, 'map_name is required'
    if mode in ('navigation', 'squad_navigation'):
        loops = mission.get('loops', 1)
        if not isinstance(loops, int) or loops < 1:
            return False, 'loops must be a positive integer'
    if mode == 'navigation':
        robots = mission.get('robots', 1)
        index = mission.get('robot_index', 0)
        if not isinstance(robots, int) or robots < 1:
            return False, 'robots must be a positive integer'
        if not isinstance(index, int) or not (0 <= index < robots):
            return False, 'robot_index must be in [0, robots)'
    return True, 'ok'


def expand_squad(mission, n=2):
    """Expand a squad_navigation mission into n per-robot navigation missions.

    Each robot gets the same map_name/loops and a distinct robot_index; the
    contiguous half-split itself happens later, inside each controller.
    """
    return [
        {'mode': 'navigation',
         'map_name': mission['map_name'],
         'loops': mission.get('loops', 1),
         'robots': n,
         'robot_index': i}
        for i in range(n)
    ]


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
def _fmt_pose(pose):
    """[x, y, yaw] -> compact 2-decimal flow list, e.g. '[0.26, 3.52, -1.57]'."""
    return '[%.2f, %.2f, %.2f]' % (float(pose[0]), float(pose[1]), float(pose[2]))


def save_perimeter(path, map_name, dock, waypoints, loops=1):
    """Write the perimeter file. `dock` and each waypoint are [x, y, yaw];
    all values are written rounded to 2 decimals. Hand-rendered (rather than
    yaml.safe_dump) so the poses stay on one compact line each:

        map: warehouse
        frame_id: map
        loops: 1
        dock:
          [0.26, 3.52, -1.57]
        waypoints:
        - [0.04, -1.13, -1.60]
    """
    lines = ['map: %s' % map_name,
             'frame_id: map',
             'loops: %d' % int(loops),
             'dock:',
             '  %s' % _fmt_pose(dock or [0.0, 0.0, 0.0]),
             'waypoints:']
    lines += ['- %s' % _fmt_pose(w) for w in waypoints]
    with open(path, 'w') as f:
        f.write('\n'.join(lines) + '\n')


def load_perimeter(path):
    """Read the perimeter file. Returns dock as [x, y, yaw] and waypoints as a
    list of [x, y, yaw]."""
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
    SEARCH_SPIN = auto()
    SEARCH_PATROL = auto()
    APPROACHING = auto()
    ARRIVED = auto()
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
    SEARCH_READY = auto()
    SPIN_DONE = auto()
    PERSON_FOUND = auto()
    PERSON_REACHED = auto()
    DOCK_REACHED = auto()
    NAV_CLOSED = auto()
    ADVANCE = auto()
    ABORT = auto()
    ERROR = auto()
    RESET = auto()          # operator 'reset': clear a fault back to idle


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
    (Phase.CLOSE_MAPPING, Event.SLAM_CLOSED): Phase.IDLE,
    (Phase.GOALPOINT_COLLECTION, Event.OPERATOR_DONE): Phase.GOAL_POINTS_SAVED,
    (Phase.GOAL_POINTS_SAVED, Event.ADVANCE): Phase.IDLE,
    (Phase.START_NAVIGATION, Event.NAV_READY): Phase.PERIMETER,
    (Phase.PERIMETER, Event.LOOPS_DONE): Phase.PERIMETER_COMPLETED,
    # Abort mid-patrol: skip the remaining loops and go straight home. The
    # node cancels the live waypoint goal first; here we just redirect.
    (Phase.PERIMETER, Event.ABORT): Phase.RETURN_TO_DOCK,
    (Phase.PERIMETER_COMPLETED, Event.ADVANCE): Phase.RETURN_TO_DOCK,
    # find_person shares navigation's startup (same perimeter/map/localization
    # setup) and only forks once Nav2 is ready: spin in place first, fall back
    # to walking the perimeter as a search route, then close in once seen.
    (Phase.START_NAVIGATION, Event.SEARCH_READY): Phase.SEARCH_SPIN,
    (Phase.SEARCH_SPIN, Event.PERSON_FOUND): Phase.APPROACHING,
    (Phase.SEARCH_SPIN, Event.SPIN_DONE): Phase.SEARCH_PATROL,
    (Phase.SEARCH_PATROL, Event.PERSON_FOUND): Phase.APPROACHING,
    (Phase.SEARCH_SPIN, Event.ABORT): Phase.RETURN_TO_DOCK,
    (Phase.SEARCH_PATROL, Event.ABORT): Phase.RETURN_TO_DOCK,
    (Phase.APPROACHING, Event.ABORT): Phase.RETURN_TO_DOCK,
    (Phase.APPROACHING, Event.PERSON_REACHED): Phase.ARRIVED,
    # No RETURN_TO_DOCK here - the robot stays put near the person. Jump
    # straight to CLOSE_NAVIGATION, which only pauses the Nav2/AMCL lifecycle
    # (no goal sent) before returning to IDLE.
    (Phase.ARRIVED, Event.ADVANCE): Phase.CLOSE_NAVIGATION,
    (Phase.RETURN_TO_DOCK, Event.DOCK_REACHED): Phase.DOCKED,
    (Phase.DOCKED, Event.ADVANCE): Phase.CLOSE_NAVIGATION,
    (Phase.CLOSE_NAVIGATION, Event.NAV_CLOSED): Phase.IDLE,
    # Operator 'reset' recovers a held fault straight to idle. No entry from
    # any other phase, so RESET is a no-op unless we're actually faulted.
    (Phase.FAULT, Event.RESET): Phase.IDLE,
}


def next_phase(phase, event):
    """Pure transition. ERROR -> FAULT from anywhere; unmapped pair is a no-op."""
    if event == Event.ERROR:
        return Phase.FAULT
    return _TRANSITIONS.get((phase, event), phase)
