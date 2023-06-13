from typing import Dict
from rclpy.time import Time, Duration
from shapely import Polygon, Point
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
import math

# This import is necessary to use buffer.transform() conversions
import tf2_geometry_msgs

from geometry_msgs.msg import Polygon as PolygonMsg
from geometry_msgs.msg import Point32, PolygonStamped, Vector3, Vector3Stamped
from visualization_msgs.msg import Marker, MarkerArray
from py_tracker_msgs.msg import PyTrackerArrayMsg, PyTrackerMsg
from std_msgs.msg import Header

from rclpy.impl import rcutils_logger

from .TrackedObject import TrackedObject

logger = rcutils_logger.RcutilsLogger(name="message_utils")


# Util functions
def get_angles(polygon: Polygon, robot_x, robot_y):
    angles = []
    for p in polygon.exterior.coords:
        xdiff = p[0] - robot_x
        ydiff = p[1] - robot_y
        angle = math.degrees(math.atan(ydiff / xdiff))
        angles.append(angle)
    return angles


async def transform_polygon(buffer: Buffer, polygon: PolygonStamped, frame: str):
    """
    tf2 doesn't yet support transformations of polygons, so we have to transform each point individually
    """
    points = polygon.polygon.points
    vector3s = [Vector3Stamped(vector=Vector3(x=point.x, y=point.y, z=0.0), header=polygon.header) for point in points]
    # ROS also doesn't have a transform_async method, so we do that ourselves too
    do_transform = buffer.registration.get(type(vector3s[0]))
    transform = await buffer.lookup_transform_async(frame, polygon.header.frame_id, polygon.header.stamp)
    transformed_vectors = [do_transform(vector3, transform) for vector3 in vector3s]
    transformed_points = [Point32(x=vector3.vector.x, y=vector3.vector.y) for vector3 in transformed_vectors]
    return PolygonStamped(polygon=PolygonMsg(points=transformed_points), header=polygon.header)


async def create_warnings_msg(
    objects: Dict[str, TrackedObject], buffer: Buffer, map_frame: str, base_frame: str, curtain_msg: PolygonStamped
):
    """
    create & publish object warning array
    """
    # Grab position of robot on map
    try:
        # Set the timeout shortish; when running this node with bagged data, somtimes it just stops accepting TF data
        # for a while. In those cases, we want to bail out quickly to avoid freezing the rest of the node.
        # However, often the buffer just organically needs time to catch up
        t = await buffer.lookup_transform_async(map_frame, base_frame, Time.from_msg(curtain_msg.header.stamp))
    except TransformException as e:
        logger.warning("PyTracker failed to get robot transform: " + str(e), throttle_duration_sec=5)
        return
    robot_x = t.transform.translation.x
    robot_y = t.transform.translation.y
    # Transform polygon to robot frame for filtering publications
    curtain_in_map = curtain_msg  # Temporarily remove this transform for performance testing
    curtain_in_map: PolygonStamped = await transform_polygon(buffer, curtain_msg, map_frame)
    curtain_poly = Polygon([(point.x, point.y) for point in curtain_in_map.polygon.points])

    # Put together warnings array, filtering tracked objects by curtain
    # Alsos transform format from cartesian to polar per project requirements
    warning_array = PyTrackerArrayMsg()
    warning_array.header = curtain_msg.header
    for id, obj in objects.items():
        if obj.polygon.intersects(curtain_poly):
            cur_obs = PyTrackerMsg()
            cur_obs.object_id = id
            angles = get_angles(obj.polygon, robot_x, robot_y)
            cur_obs.min_angle = min(angles)
            cur_obs.max_angle = max(angles)
            cur_obs.distance = obj.polygon.distance(Point(robot_x, robot_y))
            cur_obs.time_created = Time.to_msg(obj.initializedTime)
            cur_obs.is_dynamic = obj.isDynamic

            warning_array.warnings.append(cur_obs)
    return warning_array


# Makes a cylinder at the centroid of each shape
# Dynamic:  Green
# Static:   Orange
def create_marker_msg(objects: Dict[str, TrackedObject], header: Header):
    increment = 1
    marker_array = MarkerArray()
    clear_marker = Marker()
    clear_marker.header = header
    clear_marker.id = 0
    clear_marker.action = Marker.DELETEALL
    marker_array.markers.append(clear_marker)

    for obj in objects.values():
        if obj.disappearedFrames >= 1:
            continue
        marker = Marker()
        marker.type = 3
        marker.header = header
        marker.scale.x = 0.35
        marker.scale.y = 0.35
        marker.scale.z = 0.75
        marker.color.a = 1.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        if obj.isDynamic:
            marker.color.r = 0.012
            marker.color.g = 0.675
            marker.color.b = 0.075
        else:
            marker.color.r = 0.988
            marker.color.g = 0.651
            marker.color.b = 0.012
        marker.id = increment
        increment = increment + 1
        marker.pose.position.x = obj.polygon.centroid.x
        marker.pose.position.y = obj.polygon.centroid.y
        marker.pose.position.z = 0.0
        marker_array.markers.append(marker)
    return marker_array
