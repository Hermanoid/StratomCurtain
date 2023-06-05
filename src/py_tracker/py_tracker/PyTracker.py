import rclpy
from shapely import Polygon
import numpy as np
from rclpy.node import Node
from collections import OrderedDict
from costmap_converter_msgs.msg import ObstacleArrayMsg, ObstacleMsg
from scipy.spatial import distance as dist
from typing import List

class PyTracker(Node):
   
    def __init__(self):
        super().__init__('py_tracker_node', parameter_overrides=[])

        #ID to assign to the object, dictionary to keep track of mapped objects ID to centroid, number of consecutive frames marked dissapeeared
        self.nextObjectID = 1; 
        self.objects = OrderedDict()
        self.disappeared = OrderedDict()
        self.robot_x = 0
        self.robot_y = 0

        #Maximum number of frames the object can be undetectable for
        self.maxDisappeared = 50

        #Create the subscription to the topic
        self.create_subscription(
            ObstacleArrayMsg,
            'costmap_obstacles',
            self.listener_callback,
            10
        )

    def register(self, centroid):
        self.objects[self.nextObjectID] = centroid
        self.disappeared[self.nextObjectID] = 0
        self.nextObjectID += 1

    def deregister(self, objectID):
        del self.objects[objectID]
        del self.disappeared[objectID]

    def update(self, inputCentroids):
        if(len(inputCentroids) == 0):
            for objectID in self.disappeared.keys():
                self.disappeared[objectID] += 1

                if self.disappeared[objectID] > self.maxDisappeared:
                    self.deregister(objectID)
            
            return self.objects
        

        if len(self.objects) == 0:
            for centroid in inputCentroids:
                self.register(centroid)

        else:
            objectIDs = list(self.objects.keys())
            objectCentroids = list(self.objects.values())

            D = dist.cdist(np.array(objectCentroids), inputCentroids)

            rows = D.min(axis = 1).argsort()

            cols = D.argmin(axis=1)[rows]

            usedRows = set()
            usedCols = set()

            for(row, col) in zip(rows, cols):
                if row in usedRows or usedCols:
                    continue

                objectID = objectIDs[row]
                self.objects[objectID] = inputCentroids[col]
                self.disappeared[objectID] = 0

                usedRows.add(row)
                usedCols.add(col)
            
            unusedRows = set(range(0, D.shape[0])).difference(usedRows)
            unusedCols = set(range(0, D.shape[1])).difference(usedCols)

            if D.shape[0] >= D.shape[1]:
                for row in unusedRows:
                    objectID = objectIDs[row]
                    self.disappeared[objectID] += 1

                    if self.disappeared[objectID] > self.maxDisappeared:
                        self.deregister(objectID)
            else:
                for col in unusedCols:
                    self.register(inputCentroids[col])

            return self.objects
    
    def listener_callback(self,msg:ObstacleArrayMsg):
        obsArr: List[ObstacleMsg] = list(msg.obstacles)
        centroids = []
        for obstacle in obsArr:
            points = [(point.x, point.y) for point in obstacle.polygon.points]
            centroid = Polygon(shell= points).centroid
            centroids.append((centroid.x, centroid.y))
        self.update(centroids)

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