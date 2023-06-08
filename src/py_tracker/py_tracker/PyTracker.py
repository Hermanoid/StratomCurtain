import rclpy
from enum import Enum
from shapely import Polygon, Point
import numpy as np
from rclpy.node import Node
from rclpy.time import Duration
from collections import OrderedDict
from costmap_converter_msgs.msg import ObstacleArrayMsg, ObstacleMsg
from rcl_interfaces.msg import SetParametersResult
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
import math

from py_tracker_msgs.msg import PyTrackerArrayMsg, PyTrackerMsg
from scipy.spatial import distance as dist
from typing import List
import cv2

from costmap_converter_msgs.msg import ObstacleArrayMsg, ObstacleMsg
from rcl_interfaces.msg import SetParametersResult

# Import the Polygon message type as PolygonMsg because we already have the shapely.Polygon name taken
from geometry_msgs.msg import Polygon as PolygonMsg
from geometry_msgs.msg import Point32
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header

VIZ_PIXELS_PER_METER = 20
VIZ_FRAME_SIZE = 350


class TrackState(Enum):
    Static = 0
    WaitingForDynamic = 1
    Dynamic = 2


class TrackedObject:
    def __init__(self, polygon: Polygon):
        self.polygon: Polygon = polygon
        self.disappearedFrames = 0
        self.state: TrackState = TrackState.Static
        self.stateStartTime = None
        self.lastTrackedTime = None
        self.isDynamic = False
        self.velocity_rolling = np.array((0, 0))
        self.initializedTime = None

    def updateState(self, newState: TrackState, nowtime):
        self.state = newState
        self.stateStartTime = nowtime
        self.isDynamic = newState == TrackState.Dynamic


class PyTracker(Node):
    def __init__(self):
        super().__init__("py_tracker_node", parameter_overrides=[], allow_undeclared_parameters=True)
        # ID to assign to the object, dictionary to keep track of mapped objects ID to centroid, number of consecutive frames marked dissapeeared
        self.nextObjectID = 1
        self.objects: OrderedDict[int, TrackedObject] = OrderedDict()
        self.robot_x = 0
        self.robot_y = 0

        # How fast must the object be moving to be considered dynamic (should be m/s)
        self.declare_parameter("dynamic_movement_speed", 0.3)
        # How long must the object continue moving to be considered dynamic
        self.declare_parameter("dynamic_time_threshold", 0.75)
        # After an object has stopped moving, how long do we wait before dropping it back to static
        # Negative values will be interpretted as "never drop back to static"
        self.declare_parameter("dynamic_memory_time", 10.0)
        # Higher values will prioritize instantaneous velocity and deprioritize the rolling average
        self.declare_parameter("velocity_update_rate", 0.2)
        # Maximum number of frames the object can be undetectable for before getting dropped
        self.declare_parameter("max_disapeared_frames", 30)
        # ROS2 doesn't allow any complex types, including a nested array like [[pt1.x, pt1.y], [pt2.x, pt2.y], ...]
        # so for this polygon we have to get creative with a 1d array formatted [pt1.x, pt1.y, pt2.x, pt2.y, ...]
        self.declare_parameter("curtain_boundary", [0.0, 10.0, -10.0, -10.0, 10.0, -10.0])

        self.declare_parameter("curtain_publish_period", 1.0)
        self.declare_parameter("obstacles_sub_topic", "costmap_obstacles")
        self.declare_parameter("marker_pub_topic", "dynamic_marker")
        self.declare_parameter("curtain_pub_topic", "curtain")
        self.declare_parameter("warnings_pub_topic", "warnings")

        self.poly_timer = None
        self.poly_pub = None
        self.marker_pub = None
        self.warnings_pub = None
        self.obstacles_sub = None

        self.update_parameters(None)
        self.add_on_set_parameters_callback(self.update_parameters)

        # Setup for finding robot position
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def update_parameters(self, _):
        self.dynamic_movement_speed = self.get_parameter("dynamic_movement_speed").get_parameter_value().double_value
        self.dynamic_time_threshold = self.get_parameter("dynamic_time_threshold").get_parameter_value().double_value
        self.dynamic_memory_time = self.get_parameter("dynamic_memory_time").get_parameter_value().double_value
        self.velocity_update_rate = self.get_parameter("velocity_update_rate").get_parameter_value().double_value
        self.max_disappeared = 50

        boundary_array = self.get_parameter("curtain_boundary").get_parameter_value().double_array_value
        # This big-brain one-liner grabs every other point for x, every other point (shifted one) for y, and pairs them together
        boundary_points = list(zip(boundary_array[::2], boundary_array[1::2]))
        self.curtain_boundary = Polygon(shell=boundary_points)

        # Grab the updated curtain publish rate, destroy the old timer, and create a new one with the new rate.
        # ROS does not allow changing the period of an preexisting timer.
        pub_rate = self.get_parameter("curtain_publish_period").get_parameter_value().double_value
        if not self.poly_timer or pub_rate != self.poly_timer.timer_period_ns / 1000000000.0:
            if self.poly_timer:
                self.poly_timer.destroy()
            self.poly_timer = self.create_timer(pub_rate, self.publish_curtain)

        self.poly_pub = self.update_publisher(self.poly_pub, PolygonMsg, "curtain_pub_topic", 1)
        self.marker_pub = self.update_publisher(self.marker_pub, MarkerArray, "marker_pub_topic", 10)
        self.warnings_pub = self.update_publisher(self.warnings_pub, PyTrackerArrayMsg, "warnings_pub_topic", 10)

        obstacles_sub_topic = self.get_parameter("obstacles_sub_topic").get_parameter_value().string_value
        if not self.obstacles_sub or obstacles_sub_topic != self.obstacles_sub.topic:
            if self.obstacles_sub:
                self.obstacles_sub.destroy()
            self.obstacles_sub = self.create_subscription(ObstacleArrayMsg, obstacles_sub_topic, self.listener_callback, 10)
        return SetParametersResult(successful=True)

    def update_publisher(self, publisher, type, parameter, qos):
        """
        Similar to the timer object, ROS does not allow changing the topic of publishers on existinig publisher objects
        Thus, we need to destroy the old object and create it anew
        """
        new_topic_name = self.get_parameter(parameter).get_parameter_value().string_value
        if not publisher or new_topic_name != publisher.topic:
            if publisher:
                publisher.destroy()
            return self.create_publisher(type, new_topic_name, qos)
        else:
            return publisher

    def publish_curtain(self):
        # This one-line lad converters each point of the (exterior of the) curtain polygon into Point32 messages,
        # and packages those Point32s into a geometry_msgs.msg.Polygon type

        self.poly_pub.publish(PolygonMsg(points=[Point32(x=x, y=y) for x, y, in self.curtain_boundary.exterior.coords]))

    # Creates a unique ID for a polygon
    def register(self, polygon, header: Header):
        self.objects[self.nextObjectID] = TrackedObject(polygon)
        self.objects[self.nextObjectID].initializedTime = header.stamp.sec + header.stamp.nanosec / 1000000000.0
        self.nextObjectID += 1

    # Removes a polygon from list of tracked polygons
    def deregister(self, objectID):
        del self.objects[objectID]

    # create & publish object warning array
    def create_pytracker_msg(self):
        warning_array = PyTrackerArrayMsg()
        for key, value in self.objects.items():
            if value.polygon.intersects(self.curtain_boundary):
                cur_obs = PyTrackerMsg()
                cur_obs.object_id = key
                cur_obs.min_angle = self.get_min_angle(value.polygon)
                cur_obs.max_angle = self.get_max_angle(value.polygon)
                cur_obs.distance = value.polygon.distance(Point(self.robot_x, self.robot_y))
                # cur_obs.timestamp =
                # cur_obs.frame_id =
                cur_obs.is_dynamic = value.isDynamic

                # Add object to msg array ?
                warning_array.warnings.append(cur_obs)
        self.warnings_pub.publish(warning_array)

    def update(self, inputPolygons: List[Polygon], header: Header):
        """
        This update function runs on each frame of polygons, tracks them, detects motion, and publishes results
        """
        # Grab position of robot on map
        try:
            t = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time(), timeout=Duration(seconds=1))
        except TransformException as e:
            self.get_logger().warning("PyTracker failed to get robot transform: " + str(e), throttle_duration_sec=5)
            return
        self.robot_x = t.transform.translation.x
        self.robot_y = t.transform.translation.y
        # If no polygons come in, start dissapearing all tracks
        if len(inputPolygons) == 0:
            for objectID in list(self.objects.keys()):
                self.objects[objectID].disappearedFrames += 1
                if self.objects[objectID].disappearedFrames > self.max_disappeared:
                    self.deregister(objectID)
            return self.objects

        # If we have polygons coming in and none tracked, register all new polygons
        if len(self.objects) == 0:
            for polygon in inputPolygons:
                self.register(polygon, header)
        # Otherwise, start doing centroid matching

        else:
            # Attempt to associate object IDs
            objectIDs = list(self.objects.keys())

            # Calculate centroids for polygons from both this frame and the last
            objectCentroids = [(obj.polygon.centroid.x, obj.polygon.centroid.y) for obj in self.objects.values()]
            inputCentroids = [(poly.centroid.x, poly.centroid.y) for poly in inputPolygons]

            # This gets a 2D matrix of all distances between all centroids and basically does the Hungarian Algorithm
            # to find the lowest-cost (lowest-distance) matching between centroids from the last frame to this frame
            D = dist.cdist(np.array(objectCentroids), np.array(inputCentroids))
            rows = D.min(axis=1).argsort()
            cols = D.argmin(axis=1)[rows]

            usedRows = set()
            usedCols = set()

            # Use the matching to update our tracked objects dictionary
            # And, for each matched polygon, look for motion to detect static/dynamic
            for row, col in zip(rows, cols):
                if row in usedRows or col in usedCols:
                    continue

                objectID = objectIDs[row]
                newPoly = inputPolygons[col]
                self.detect_dynamic(objectID, newPoly)
                self.objects[objectID].polygon = newPoly
                self.objects[objectID].disappearedFrames = 0

                usedRows.add(row)
                usedCols.add(col)

            unusedRows = set(range(0, D.shape[0])).difference(usedRows)
            unusedCols = set(range(0, D.shape[1])).difference(usedCols)
            # Deal with lost or disappeared objects
            # If there are more polygons tracked (shape[0]) than detected (shape[1]) this frame, start dissapearing
            if D.shape[0] >= D.shape[1]:
                for row in unusedRows:
                    objectID = objectIDs[row]
                    self.objects[objectID].disappearedFrames += 1

                    if self.objects[objectID].disappearedFrames > self.max_disappeared:
                        self.deregister(objectID)
            # Else if there were more polygons detected than tracked this frame, register the new ones
            else:
                for col in unusedCols:
                    self.register(inputPolygons[col], header)

            return self.objects

    def detect_dynamic(self, objectID, newPoly: Polygon):
        """
        For an object with a given object ID, look for movement between the remembered location and the
        provided new polygon location, do some filtering, and finally mark this object static/dynamic.
        """
        nowtime = self.get_clock().now()

        def getElapsed(oldTime):
            return (nowtime.nanoseconds - oldTime.nanoseconds) / 1000000000.0

        obj = self.objects[objectID]
        oldPoly: Polygon = obj.polygon

        # if the lastTrackedTime variable has been set (i.e., this isn't our first frame seeing this object)
        if obj.lastTrackedTime:
            deltatime = getElapsed(obj.lastTrackedTime)
            # Technically, this isn't instantaneous velocity but the average velocity since the last frame
            instant_velocity = np.array((newPoly.centroid.x - oldPoly.centroid.x, newPoly.centroid.y - oldPoly.centroid.y)) / deltatime
            # Perform a rolling average of velocity to filter out shaky back-and-forth movement
            obj.velocity_rolling = obj.velocity_rolling * (1 - self.velocity_update_rate) + instant_velocity * self.velocity_update_rate
            # The magnitude of the (rolling average) velocity vector is speed, in m/s
            speed = np.linalg.norm(obj.velocity_rolling)
            moving = speed >= self.dynamic_movement_speed

            # This state update tree handles state transitions and the time_threshold filter,
            # which requires that an object be moving constantly for an amount of time before it becomes dynamic.
            # Once an object becomes dynamic, it also handles switching back to static
            #  after a specified period of not moving.
            if moving and obj.state == TrackState.Static:
                obj.updateState(TrackState.WaitingForDynamic, nowtime)
            elif obj.state == TrackState.WaitingForDynamic:
                if moving and getElapsed(obj.stateStartTime) > self.dynamic_time_threshold:
                    obj.updateState(TrackState.Dynamic, nowtime)
                elif not moving:
                    obj.updateState(TrackState.Static, nowtime)
            elif obj.state == TrackState.Dynamic:
                if moving:
                    obj.updateState(TrackState.Dynamic, nowtime)  # Reset dynamic timer
                elif self.dynamic_memory_time >= 0 and getElapsed(obj.stateStartTime) > self.dynamic_memory_time:
                    obj.updateState(TrackState.Static, nowtime)

        obj.lastTrackedTime = nowtime

    def listener_callback(self, msg: ObstacleArrayMsg):
        obsArr: List[ObstacleMsg] = list(msg.obstacles)
        # Gets Shapely polygons from the polygons in the input message and filters bad polygons
        polygons = []
        bad_points = []
        for obstacle in obsArr:
            points = [(point.x, point.y) for point in obstacle.polygon.points]
            if len(points) < 3:
                bad_points += points
                continue
            polygon = Polygon(shell=points)
            polygons.append(polygon)
        self.update(polygons, msg.header)
        self.visualize_tracks(polygons, bad_points)
        self.visualize_markers(msg)
        self.create_pytracker_msg()

    def visualize_tracks(self, inputPolygons, bad_points):
        offset_y = offset_x = VIZ_FRAME_SIZE / 2

        def pixelify(point):
            return (int(offset_x + VIZ_PIXELS_PER_METER * point[0]), int(offset_y - VIZ_PIXELS_PER_METER * point[1]))

        # Draw received polygons in the left image
        input_im = np.zeros((VIZ_FRAME_SIZE, VIZ_FRAME_SIZE, 3))
        for poly in inputPolygons:
            poly_coords = [pixelify(pt) for pt in poly.exterior.coords]
            cv2.fillPoly(input_im, [np.array(poly_coords)], color=(0.75, 0.25, 0))
        for point in bad_points:
            cv2.circle(input_im, pixelify(point), 3, (0, 0, 0.75), -1)

        # Draw tracked polygons, colored by their dynamic status
        track_im = np.zeros((VIZ_FRAME_SIZE, VIZ_FRAME_SIZE, 3))
        for obj in self.objects.values():
            poly_coords = [pixelify(pt) for pt in obj.polygon.exterior.coords]
            color = (0.075, 0.675, 0.012) if obj.isDynamic else (0.012, 0.651, 0.988)
            cv2.fillPoly(track_im, [np.array(poly_coords)], color=color)

        # Overlay some text about tracking status
        for objectID, obj in zip(self.objects.keys(), self.objects.values()):
            centroid_pt = pixelify((obj.polygon.centroid.x, obj.polygon.centroid.y))
            cv2.putText(
                track_im, f"{objectID}, d{obj.disappearedFrames}", centroid_pt, cv2.FONT_HERSHEY_PLAIN, 0.5, (1, 1, 1), 1, cv2.LINE_AA
            )

        # Combine input and tracked (left and right) images and show
        combined_im = np.hstack((input_im, track_im))
        cv2.imshow("Tracks", combined_im)
        cv2.waitKey(1)

    # Makes a cylinder at the centroid of each shape
    # Dynamic:  Green
    # Static:   Orange
    def visualize_markers(self, msg: ObstacleArrayMsg):
        increment = 1
        marker_array = MarkerArray()
        clear_marker = Marker()
        clear_marker.id = 0
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)

        for obj in self.objects.values():
            marker = Marker()
            marker.type = 3
            marker.header.stamp = msg.header.stamp
            marker.header.frame_id = "map"
            marker.scale.x = 0.35
            marker.scale.y = 0.35
            marker.scale.z = 0.75
            marker.color.a = 1.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            if obj.isDynamic:
                marker.color.r = 0.988
                marker.color.g = 0.651
                marker.color.b = 0.012
            else:
                marker.color.r = 0.012
                marker.color.g = 0.675
                marker.color.b = 0.075
            marker.id = increment
            increment = increment + 1
            marker.pose.position.x = obj.polygon.centroid.x
            marker.pose.position.y = obj.polygon.centroid.y
            marker.pose.position.z = 0.0
            # self.marker_publisher_.publish(marker)
            marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)

    def get_angles(self, polygon: Polygon):
        angles = []
        for p in polygon.exterior.coords:
            ydiff = p[1] - self.robot_y
            xdiff = p[0] - self.robot_x
            angle = math.degrees(math.atan(ydiff / xdiff))
            angles.append(angle)
        return angles

    def get_min_angle(self, polygon):
        return min(self.get_angles(polygon))

    def get_max_angle(self, polygon):
        return max(self.get_angles(polygon))


def main(args=None):
    rclpy.init(args=args)
    try:
        py_tracker_node = PyTracker()
        rclpy.spin(py_tracker_node)
    except KeyboardInterrupt:
        print("PyTracker was terminated by a KeyboardInterrupt")


if __name__ == "__main__":
    main()
