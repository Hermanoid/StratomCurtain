from enum import Enum
import rclpy
from shapely import Polygon
import numpy as np
from rclpy.node import Node
from collections import OrderedDict
from costmap_converter_msgs.msg import ObstacleArrayMsg, ObstacleMsg
from rcl_interfaces.msg import SetParametersResult

# from py_tracker_msg import PyTrackerArrayMsg, PyTrackerMsg
from scipy.spatial import distance as dist
from typing import List
import cv2

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

        self.update_parameters(None)
        self.add_on_set_parameters_callback(self.update_parameters)
        # Create a publisher to a topic (be able to change topic name?)
        # self.publisher_ = self.create_publisher(
        #     PyTrackerArrayMsg,
        #     'warning_messages',
        #     10
        # )

    def update_parameters(self, _):
        self.dynamic_movement_speed = self.get_parameter("dynamic_movement_speed").get_parameter_value().double_value
        self.dynamic_time_threshold = self.get_parameter("dynamic_time_threshold").get_parameter_value().double_value
        self.dynamic_memory_time = self.get_parameter("dynamic_memory_time").get_parameter_value().double_value
        self.velocity_update_rate = self.get_parameter("velocity_update_rate").get_parameter_value().double_value
        return SetParametersResult(successful=True)

    # Creates a unique ID for a polygon
    def register(self, polygon):
        self.objects[self.nextObjectID] = TrackedObject(polygon)
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
        if len(inputPolygons) == 0:
            for objectID in list(self.objects.keys()):
                self.objects[objectID].dissappearedFrames += 1
                if self.objects[objectID].dissappearedFrames > self.maxDisappeared:
                    self.deregister(objectID)
            return self.objects

        if len(self.objects) == 0:
            for polygon in inputPolygons:
                self.register(polygon)

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
                    self.register(inputPolygons[col])

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
        self.update(polygons)
        self.visualize_tracks(polygons, bad_points)

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
