"""Mission JSON validation helpers (ROS-free)."""
from core_task_controller.function import validate_mission


def validate_and_explain(mission):
    """Validate a mission dict and return structured result.

    Args:
        mission: Dict with keys {mode, map_name, loops?}

    Returns:
        {
            "valid": bool,
            "reason": str,  # empty if valid, error message if invalid
            "mission": dict | None  # original mission if valid, None if invalid
        }
    """
    ok, reason = validate_mission(mission)
    return {
        "valid": ok,
        "reason": "" if ok else reason,
        "mission": mission if ok else None,
    }
