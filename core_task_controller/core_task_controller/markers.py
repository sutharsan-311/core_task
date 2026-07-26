"""RViz MarkerArray for perimeter waypoints, shared by the collector (shows
points as they are clicked) and the navigator (shows the loaded perimeter while
driving). ROS-dependent, so kept out of the ROS-free function.py.

Published latched on /waypoints; the nav2.rviz MarkerArray display renders it.
"""
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


def waypoint_markers(waypoints, frame='map', ns='perimeter',
                     color=(0.1, 0.85, 0.2)):
    """MarkerArray: one sphere per [x, y, yaw] waypoint plus a line strip
    through them. A leading DELETEALL clears any previous, larger set so
    re-clicks during collection don't leave orphaned markers behind.
    """
    r, g, b = color
    arr = MarkerArray()

    clear = Marker()
    clear.action = Marker.DELETEALL
    arr.markers.append(clear)

    for i, wp in enumerate(waypoints):
        m = Marker()
        m.header.frame_id = frame
        m.ns = ns
        m.id = i
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = float(wp[0])
        m.pose.position.y = float(wp[1])
        m.pose.position.z = 0.1
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.25
        m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 1.0
        arr.markers.append(m)

    if len(waypoints) >= 2:
        line = Marker()
        line.header.frame_id = frame
        line.ns = ns + '_line'
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.05
        line.color.r, line.color.g, line.color.b, line.color.a = r, g, b, 0.6
        line.pose.orientation.w = 1.0
        line.points = [Point(x=float(w[0]), y=float(w[1]), z=0.1)
                       for w in waypoints]
        arr.markers.append(line)

    return arr
