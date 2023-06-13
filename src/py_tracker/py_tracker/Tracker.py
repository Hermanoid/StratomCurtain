from collections import OrderedDict
from typing import List
from shapely import Polygon
from std_msgs.msg import Header
from scipy.spatial import distance as dist
import numpy as np
from rclpy.time import Time
from rclpy.node import Node

from .TrackedObject import TrackState, TrackedObject
from rclpy.impl import rcutils_logger


logger = rcutils_logger.RcutilsLogger(name="tracker")


class Tracker:
    def __init__(self, node: Node):
        """
        Creates a new Tracker object. Uses the node for parameter access and to get the node's ROS clock.
        """
        self.nextObjectID = 0
        self.objects: OrderedDict[int, TrackedObject] = OrderedDict()
        # How fast must the object be moving to be considered dynamic (should be m/s)
        node.declare_parameter("dynamic_movement_speed", 0.3)
        # How long must the object continue moving to be considered dynamic
        node.declare_parameter("dynamic_time_threshold", 0.75)
        # After an object has stopped moving, how long do we wait before dropping it back to static
        # Negative values will be interpretted as "never drop back to static"
        node.declare_parameter("dynamic_memory_time", 10.0)
        # Higher values will prioritize instantaneous velocity and deprioritize the rolling average
        node.declare_parameter("velocity_update_rate", 0.2)
        # Maximum number of frames the object can be undetectable for before getting dropped
        node.declare_parameter("max_disapeared_frames", 30)
        self.clock = node.get_clock()
        self.update_parameters(node)

    def update_parameters(self, node: Node):
        self.dynamic_movement_speed = node.get_parameter("dynamic_movement_speed").get_parameter_value().double_value
        self.dynamic_time_threshold = node.get_parameter("dynamic_time_threshold").get_parameter_value().double_value
        self.dynamic_memory_time = node.get_parameter("dynamic_memory_time").get_parameter_value().double_value
        self.velocity_update_rate = node.get_parameter("velocity_update_rate").get_parameter_value().double_value
        self.max_disappeared = node.get_parameter("max_disapeared_frames").get_parameter_value().integer_value

    def update(self, inputPolygons: List[Polygon], header: Header):
        """
        This update function runs on each frame of polygons, tracks them, detects motion, and publishes results
        """
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

    # Creates a unique ID for a polygon
    def register(self, polygon, header: Header):
        self.objects[self.nextObjectID] = TrackedObject(polygon, Time.from_msg(header.stamp))
        self.nextObjectID += 1

    # Removes a polygon from list of tracked polygons
    def deregister(self, objectID):
        del self.objects[objectID]

    def detect_dynamic(self, objectID, newPoly: Polygon):
        """
        For an object with a given object ID, look for movement between the remembered location and the
        provided new polygon location, do some filtering, and finally mark this object static/dynamic.
        """
        nowtime = self.clock.now()

        def getElapsed(oldTime):
            return (nowtime.nanoseconds - oldTime.nanoseconds) / 1000000000.0

        obj = self.objects[objectID]
        oldPoly: Polygon = obj.polygon

        # if the lastTrackedTime variable has been set (i.e., this isn't our first frame seeing this object)
        if obj.lastTrackedTime:
            deltatime = getElapsed(obj.lastTrackedTime)
            if deltatime == 0:
                # If the time between frames is 0, we can't calculate velocity, so skip this frame
                logger.warning("That's odd - tracker encountered a deltatime of zero", throttle_duration_sec=1.0)
                return
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
