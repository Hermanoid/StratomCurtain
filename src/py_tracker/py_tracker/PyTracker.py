import rclpy
import shapely
import numpy
from rclpy.node import Node
from costmap_converter import ObstacleArrayMsg

class PyTracker(Node):
   
   def __init__(self):
        super().__init__('py_tracker_node')
        self.subscription = self.create_subscription(
            ObstacleArrayMsg,
            'FILL IN TOPIC NAME',
            self.listener_callback,
            10
        )
        self.subscription

    
def message_callback(self,msg):
        return


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