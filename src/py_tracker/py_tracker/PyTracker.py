import rclpy
from enum import Enum
from shapely import Polygon
import numpy as np
from rclpy.node import Node
from collections import OrderedDict
from costmap_converter_msgs.msg import ObstacleArrayMsg, ObstacleMsg
from rcl_interfaces.msg import SetParametersResult
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
import math

# from py_tracker_msg import PyTrackerArrayMsg, PyTrackerMsg
from scipy.spatial import distance as dist
from typing import List
import cv2

from costmap_converter_msgs.msg import ObstacleArrayMsg, ObstacleMsg
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Polygon as PolygonMsg
from geometry_msgs.msg import Point32
from visualization_msgs.msg import Marker, MarkerArray

# from py_tracker_msg import PyTrackerArrayMsg, PyTrackerMsg

VIZ_PIXELS_PER_METER = 20
VIZ_FRAME_SIZE = 350


class TrackState(Enum):
    Static = 0
    WaitingForDynamic = 1
    Dynamic = 2


class TrackedObject:
    def __init__(self, polygon: Polygon):
        self.polygon: Polygon = polygon
        self.dissappearedFrames = 0
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
        super().__init__("py_tracker_node", parameter_overrides=[])
        # ID to assign to the object, dictionary to keep track of mapped objects ID to centroid, number of consecutive frames marked dissapeeared
        self.nextObjectID = 1
        self.objects: OrderedDict[int, TrackedObject] = OrderedDict()
        self.robot_x = 0
        self.robot_y = 0

        # Maximum number of frames the object can be undetectable for
        self.maxDisappeared = 0

        # Create the subscription to the topic
        self.create_subscription(ObstacleArrayMsg, "costmap_obstacles", self.listener_callback, 10)

        # How fast must the object be moving to be considered dynamic (should be m/s)
        self.declare_parameter("dynamic_movement_speed", 0.3)
        # How long must the object continue moving to be considered dynamic
        self.declare_parameter("dynamic_time_threshold", 0.75)
        # After an object has stopped moving, how long do we wait before dropping it back to static
        # Negative values will be interpretted as "never"
        self.declare_parameter("dynamic_memory_time", 10.0)
        # Higher values will prioritize instantaneous velocity and deprioritize the rolling average
        self.declare_parameter("velocity_update_rate", 0.2)
        # ROS doesn't allow any complex types, including arrays of points, so we have to get creative with a dict
        # self.declare_parameter("curtain_boundary", {"a": [0, 10], "b": [-10, -10], "c": [10, -10]})
        self.declare_parameters("curtain_boundary", [["a", [0.0, 10.0]], ["b", [-10.0, -10.0]], ["c", [-10.0, 10.0]]])
        self.declare_parameter("curtain_publish_rate", 1.0)

        self.poly_pub = self.create_publisher(PolygonMsg, "curtain", 1)
        self.poly_timer = None

        self.update_parameters(None)
        self.add_on_set_parameters_callback(self.update_parameters)
        # Create a publisher to a topic (be able to change topic name?)
        # self.publisher_ = self.create_publisher(
        #     PyTrackerArrayMsg,
        #     'warning_messages',
        #     10
        # )
        self.marker_publisher_ = self.create_publisher(
            MarkerArray, 
            'dynamic_obsticle_marker', 
            10
        )

        #Setup for finding robot position
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def update_parameters(self, _):
        self.dynamic_movement_speed = self.get_parameter("dynamic_movement_speed").get_parameter_value().double_value
        self.dynamic_time_threshold = self.get_parameter("dynamic_time_threshold").get_parameter_value().double_value
        self.dynamic_memory_time = self.get_parameter("dynamic_memory_time").get_parameter_value().double_value
        self.velocity_update_rate = self.get_parameter("velocity_update_rate").get_parameter_value().double_value

        boundary_points = [
            parameter.get_parameter_value().double_array for parameter in self.get_parameters_by_prefix("curtain_boundary").values()
        ]
        self.curtain_boundary = Polygon(shell=boundary_points)

        pub_rate = self.get_parameter("curtain_publish_rate").get_parameter_value().double_value
        if self.poly_timer:
            self.poly_timer.destroy()
        self.poly_timer = self.create_timer(pub_rate, self.publish_curtain)
        return SetParametersResult(successful=True)

    def publish_curtain(self):
        self.poly_pub.publish(PolygonMsg(points=[Point32(x=x, y=y) for x, y, in self.curtain_boundary.exterior.coords]))

    # Creates a unique ID for a polygon
    def register(self, polygon):
        self.objects[self.nextObjectID] = TrackedObject(polygon)
        self.objects[slef.nextObjectID] = msg.header.stamp
        self.nextObjectID += 1

    # Removes a polygon from list of tracked polygons
    def deregister(self, objectID):
        del self.objects[objectID]

    # create individual objects, add to message array. Update parameters as needed CHECK!
    # def create_object_msg(self):
    #     cur_object = PyTrackerMsg()
    #     # populate object fields ...
    #     # Add object to msg array
    #     self.msg.warnings.append(cur_object)

    # Called seperately to publish entire array to topic '/warning_messages' CHECK!
    def publish(self):
        self.publisher_.publish(self.msg)

    def update(self, inputPolygons: List[Polygon]):
        #Grab position of robot on map
        t = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
        self.robot_x = t.translation.x
        self.robot_y = t.translation.y
        if len(inputPolygons) == 0:
            for objectID in list(self.objects.keys()):
                self.objects[objectID].dissappearedFrames += 1
                if self.objects[objectID].dissappearedFrames > self.maxDisappeared:
                    self.deregister(objectID)
            return self.objects

        if len(self.objects) == 0:
            for polygon in inputPolygons:
                self.register(polygon, msg)

        else:
            # Attempt to associate object IDs
            objectIDs = list(self.objects.keys())

            objectCentroids = [(obj.polygon.centroid.x, obj.polygon.centroid.y) for obj in self.objects.values()]
            inputCentroids = [(poly.centroid.x, poly.centroid.y) for poly in inputPolygons]

            D = dist.cdist(np.array(objectCentroids), np.array(inputCentroids))
            rows = D.min(axis=1).argsort()
            cols = D.argmin(axis=1)[rows]

            usedRows = set()
            usedCols = set()

            for row, col in zip(rows, cols):
                if row in usedRows or col in usedCols:
                    continue

                objectID = objectIDs[row]
                oldPoly = self.objects[objectID].polygon
                newPoly = inputPolygons[col]
                self.detect_dynamic(objectID, oldPoly, newPoly)
                self.objects[objectID].polygon = newPoly
                self.objects[objectID].dissappearedFrames = 0

                usedRows.add(row)
                usedCols.add(col)

            unusedRows = set(range(0, D.shape[0])).difference(usedRows)
            unusedCols = set(range(0, D.shape[1])).difference(usedCols)
            # Deal with lost or disappeared objects
            if D.shape[0] >= D.shape[1]:
                for row in unusedRows:
                    objectID = objectIDs[row]
                    self.objects[objectID].dissappearedFrames += 1

                    if self.objects[objectID].dissappearedFrames > self.maxDisappeared:
                        self.deregister(objectID)
            else:
                for col in unusedCols:
                    self.register(inputPolygons[col], msg)

            return self.objects

    def detect_dynamic(self, objectID, oldPoly: Polygon, newPoly: Polygon):
        nowtime = self.get_clock().now()

        def getElapsed(oldTime):
            return (nowtime.nanoseconds - oldTime.nanoseconds) / 1000000000.0

        obj = self.objects[objectID]

        # if the lastTrackedTime variable has been set
        if obj.lastTrackedTime:
            deltatime = getElapsed(obj.lastTrackedTime)
            instant_velocity = np.array((newPoly.centroid.x - oldPoly.centroid.x, newPoly.centroid.y - oldPoly.centroid.y)) / deltatime
            obj.velocity_rolling = obj.velocity_rolling * (1 - self.velocity_update_rate) + instant_velocity * self.velocity_update_rate
            speed = np.linalg.norm(obj.velocity_rolling)
            moving = speed >= self.dynamic_movement_speed
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
        pass

    def listener_callback(self, msg: ObstacleArrayMsg):
        obsArr: List[ObstacleMsg] = list(msg.obstacles)
        # Gets the centroids and points from the polygons in the input message
        polygons = []
        bad_points = []
        for obstacle in obsArr:
            points = [(point.x, point.y) for point in obstacle.polygon.points]
            if len(points) < 3:
                bad_points += points
                continue
            polygon = Polygon(shell=points)
            polygons.append(polygon)
        self.update(polygons, msg)
        self.visualize_tracks(polygons, bad_points)
        self.visualize_markers(msg)

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
                track_im, f"{objectID}, d{obj.dissappearedFrames}", centroid_pt, cv2.FONT_HERSHEY_PLAIN, 0.5, (1, 1, 1), 1, cv2.LINE_AA
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
        self.marker_publisher_.publish(marker_array)

    def get_min_angle(self, polygon):
        minangle = None
        for p in polygon:
            ydiff = p.y - self.robot_y
            xdiff = p.x - self.robot_x
            angle = math.degrees(math.atan(ydiff/xdiff))
            if(minangle == None or angle < minangle):
                minangle = angle
        return minangle
    
    def get_max_angle(self, polygon):
        maxangle = None
        for p in polygon:
            ydiff = p.y - self.robot_y
            xdiff = p.x - self.robot_x
            angle = math.degrees(math.atan(ydiff/xdiff))
            if(maxangle == None or angle > maxangle):
                maxangle = angle
        return maxangle

def main(args=None):
    rclpy.init(args=args)
    try:
        py_tracker_node = PyTracker()
        rclpy.spin(py_tracker_node)
    except KeyboardInterrupt:
        print("PyTracker was terminated by a KeyboardInterrupt")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
