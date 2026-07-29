"""Pure vision-servo control law for the person-approach step.

Turns a target_detection report (see detection.summarize) into a drive
command: turn toward the target while it's visible and still far, stop and
report arrival once its box fills enough of the frame to call it "close". No
rclpy/torch imports here, so it unit-tests without a ROS graph or a model.
"""


def steering_command(report, arrived_area=0.35, k_angular=1.5, linear_speed=0.15):
    """Turn a detection report into (linear_x, angular_z, arrived).

    Args:
        report: a detection.summarize() dict (found, cx, cy, area).
        arrived_area: box-area fraction (0..1) that counts as "close enough".
            There's no depth sensor, so this is a proxy for distance - it
            depends on the camera's real FOV and needs a one-time field
            calibration (walk the robot to the target, read the reported
            area, tune this to match).
        k_angular: proportional gain turning toward an off-center target.
        linear_speed: constant forward crawl speed while still approaching.

    Returns:
        (linear_x, angular_z, arrived) - all zero and arrived=False when
        nothing is detected; zero velocity and arrived=True once close.
    """
    if not report['found']:
        return 0.0, 0.0, False
    if report['area'] >= arrived_area:
        return 0.0, 0.0, True
    return linear_speed, -k_angular * report['cx'], False
