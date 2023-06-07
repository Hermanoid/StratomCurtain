import rclpy
from shapely import Polygon
import numpy as np
from rclpy.node import Node
from collections import OrderedDict
from costmap_converter_msgs.msg import ObstacleArrayMsg, ObstacleMsg
from py_tracker_msg import PyTrackerArrayMsg, PyTrackerMsg
from scipy.spatial import distance as dist
from typing import List
import cv2
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
import math


VIZ_PIXELS_PER_METER = 20
VIZ_FRAME_SIZE = 300

class PyTracker(Node):
   
    def __init__(self):
        super().__init__('py_tracker_node', parameter_overrides=[])
        #Output message array
        self.msg = PyTrackerArrayMsg()
        #ID to assign to the object, dictionary to keep track of mapped objects ID to centroid, number of consecutive frames marked dissapeeared
        self.nextObjectID = 1; 
        self.objects: OrderedDict[int, Polygon] = OrderedDict()
        self.disappeared = OrderedDict()
        self.robot_x = 0
        self.robot_y = 0

        #Maximum number of frames the object can be undetectable for
        self.maxDisappeared = 0

        #Create the subscription to the topic
        self.create_subscription(
            ObstacleArrayMsg,
            'costmap_obstacles',
            self.listener_callback,
            10
        )
        #Create a publisher to a topic (be able to change topic name?)
        self.publisher_ = self.create_publisher(
            PyTrackerArrayMsg, 
            'warning_messages', 
            10
        )

        #Setup for finding robot position
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    #Creates a unique ID for a polygon
    def register(self, centroid):
        self.objects[self.nextObjectID] = centroid
        self.disappeared[self.nextObjectID] = 0
        self.nextObjectID += 1
    #Removes a polygon from list of tracked polygons
    def deregister(self, objectID):
        del self.objects[objectID]
        del self.disappeared[objectID]
    
    #Create individual objects, add to message array. Update parameters as needed CHECK!
    def create_object_msg(self):
        cur_object = PyTrackerMsg()
        # populate object fields ...
        # Add object to msg array 
        self.msg.warnings.append(cur_object)
    
    #Called seperately to publish entire array to topic '/warning_messages' CHECK!
    def publish(self):
        self.publisher_.publish(self.msg)

    #Ids and tracks polygons and detects motion
    def update(self, inputCentroids):
        #Grab position of robot on map
        t = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
        self.robot_x = t.translation.x
        self.robot_y = t.translation.y
        #Ages polygons that are being tracked
        if(len(inputCentroids) == 0):
            for objectID in self.disappeared.keys():
                self.disappeared[objectID] += 1
                if self.disappeared[objectID] > self.maxDisappeared:
                    self.deregister(objectID)
            return self.objects
        
        if len(self.objects) == 0:
            #If there were no registered objects then register all centroids
            for centroid in inputCentroids:
                self.register(centroid)

        else:
            #Attempt to associate object IDs
            objectIDs = list(self.objects.keys())

            objectCentroids = [(poly.centroid.x, poly.centroid.y) for poly in self.objects.values()]
            inputCentroids = [(poly.centroid.x, poly.centroid.y) for poly in inputPolygons]

            D = dist.cdist(np.array(objectCentroids), np.array(inputCentroids))
            rows = D.min(axis = 1).argsort()
            cols = D.argmin(axis=1)[rows]

            usedRows = set()
            usedCols = set()

            for(row, col) in zip(rows, cols):
                if row in usedRows or col in usedCols:
                    continue

                objectID = objectIDs[row]
                self.objects[objectID] = inputPolygons[col]
                self.disappeared[objectID] = 0

                usedRows.add(row)
                usedCols.add(col)
            
            unusedRows = set(range(0, D.shape[0])).difference(usedRows)
            unusedCols = set(range(0, D.shape[1])).difference(usedCols)
            #Deal with lost or disappeared objects
            if D.shape[0] >= D.shape[1]:
                for row in unusedRows:
                    objectID = objectIDs[row]
                    self.disappeared[objectID] += 1

                    if self.disappeared[objectID] > self.maxDisappeared:
                        self.deregister(objectID)
            else:
                for col in unusedCols:
                    self.register(inputPolygons[col])

            return self.objects
    
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

    def listener_callback(self,msg:ObstacleArrayMsg):
        obsArr: List[ObstacleMsg] = list(msg.obstacles)
        #Gets the centroids and points from the polygons in the input message
        polygons = []
        bad_points = []
        for obstacle in obsArr:
            points = [(point.x, point.y) for point in obstacle.polygon.points]
            if(len(points)<3):
                print("Bad polygon with not enough points")
                bad_points += points
                continue
            polygon = Polygon(shell= points)
            polygons.append(polygon)
        self.update(polygons)
        self.visualize_tracks(polygons, bad_points)

    def visualize_tracks(self, inputPolygons: List[Polygon], bad_points):
        offset_y = offset_x = VIZ_FRAME_SIZE/2
        def pixelify(point):
            return (int(offset_x + VIZ_PIXELS_PER_METER * point[0]), int(offset_y + VIZ_PIXELS_PER_METER * point[1]))
        def draw_polygons(im, polygons):
            for poly in polygons:
                poly_coords = [pixelify(pt) for pt in poly.exterior.coords]
                cv2.polylines(im, [np.array(poly_coords)], isClosed=True, color=(0, 0.75, 0), thickness=2)
        
        input_im = np.zeros((VIZ_FRAME_SIZE, VIZ_FRAME_SIZE, 3))
        draw_polygons(input_im, inputPolygons)
        for point in bad_points:
            cv2.circle(input_im, pixelify(point), 5, (0, 0, 0.75), -1)
        
        track_im = np.zeros((VIZ_FRAME_SIZE, VIZ_FRAME_SIZE, 3))
        draw_polygons(track_im, self.objects.values())
        for objectID, poly, dissappeared_frames in zip(self.objects.keys(), self.objects.values(), self.disappeared.values()):
            pass
            # cv2.putText(im, f"{objectID}, d{dissappeared_frames}", (poly.centroid.x, poly.centroid.y), cv2.FONT_HERSHEY_PLAIN,5, (1, 1, 1), 1, cv2.LINE_AA)
        combined_im = np.hstack((input_im, track_im))
        cv2.imshow("Tracks", combined_im)
        cv2.waitKey(1)
        print("Showing new track")

def main(args=None):
    rclpy.init(args=args)
    try:
        py_tracker_node = PyTracker()
        rclpy.spin(py_tracker_node)
    except KeyboardInterrupt:
        print("PyTracker was terminated by a KeyboardInterrupt")

    rclpy.shutdown()

if __name__ == '__main__':
    main()