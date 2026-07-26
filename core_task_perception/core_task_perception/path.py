"""Pure path math for the target mover (ROS-free).

Walks a point at constant speed around a closed polyline and reports its pose.
No rclpy here so the geometry unit-tests without a ROS graph.
"""
import math


def segment_lengths(points):
    """Return the length of each segment of the closed loop through points."""
    n = len(points)
    return [math.dist(points[i], points[(i + 1) % n]) for i in range(n)]


def loop_length(points):
    """Total perimeter of the closed loop."""
    return sum(segment_lengths(points))


def pose_at(points, distance):
    """Pose of a walker `distance` metres along the closed loop.

    Args:
        points: list of (x, y) vertices; the loop closes from last back to first.
        distance: arc length travelled; wraps, so any real value is valid.

    Returns:
        (x, y, yaw) - position on the loop and heading along the current segment.
    """
    n = len(points)
    segs = segment_lengths(points)
    total = sum(segs)
    if total <= 0:
        x, y = points[0]
        return (x, y, 0.0)

    d = distance % total
    for i in range(n):
        seg = segs[i]
        if d <= seg or i == n - 1:
            a = points[i]
            b = points[(i + 1) % n]
            t = 0.0 if seg == 0 else d / seg
            x = a[0] + (b[0] - a[0]) * t
            y = a[1] + (b[1] - a[1]) * t
            yaw = math.atan2(b[1] - a[1], b[0] - a[0])
            return (x, y, yaw)
        d -= seg
    # Unreachable (the loop above always returns), but keep the type stable.
    x, y = points[0]
    return (x, y, 0.0)


def ping_pong(waypoints):
    """Out-and-back loop through waypoints.

    Returns waypoints followed by the interior reversed, so the closing segment
    is an existing edge (last back to first) rather than a diagonal across the
    middle. Walking this as a closed loop retraces the open path both ways and
    never cuts through the interior.
    """
    if len(waypoints) < 2:
        return list(waypoints)
    return list(waypoints) + list(reversed(waypoints[1:-1]))
