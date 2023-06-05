import rclpy
from shapely import LineString, MultiPoint, Polygon
import numpy as np
from rclpy.node import Node
from collections import OrderedDict
from costmap_converter_msgs.msg import ObstacleArrayMsg
from scipy.spatial import distance as dist

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
        self.subscription = self.create_subscription(
            ObstacleArrayMsg,
            'converter_obstacles',
            self.listener_callback,
            10
        )

        self.subscription2 = self.create_subscription( TFMessage, 'tf', self.tf_callback, 10)


    def register(self, centroid):
        self.objects[self.nextObjectID] = centroid
        self.disappeared[self.nextObjectID] = 0
        self.nextObjectID += 1

    def deregister(self, objectID):
        del self.objects[objectID]
        del self.disappeared[objectID]

    def update(self, polygons):
        if(len(polygons) == 0):
            for objectID in list(self.disappeared.keys()):
                self.disappeared[objectID] += 1

                if self.disappeared[objectID] > self.maxDisappeared:
                    self.deregister(objectID)
            
            return self.objects
        
        #Create array of centroids from polygons
        inputCentroids = []

        for i in enumerate(polygons):
            center = polygons[i].centroid
            inputCentroids[i] = center

        if len(self.objects) == 0:
            for i in range(0, len(inputCentroids)):
                self.register(inputCentroids[i])

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
        
    def tf_callback(self, msg):
        robot_x = msg.transform.x
        robot_y = msg.transform.y

    
    def listener_callback(self,msg:ObstacleArrayMsg):
        obsArr = msg.obstacles
        polygons = []
        for i in len(obsArr):
            polygons.append((obsArr[i].polygon.centroid.x, obsArr[i].polygon.centroid.y))
        self.update(polygons)

def main(args=None):
    #unsure about this
    rclpy.init(args=args)
    #
    
    py_tracker_node = PyTracker()
    rclpy.spin(py_tracker_node)
    py_tracker_node.destroy_node()

    #unsure about this
    rclpy.shutdown()
    #

if __name__ == '__main__':
    main()