from typing import List, Optional
import rclpy

from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_service import SetParametersResult

class ConfigTest(Node):
    def __init__(self):
        super().__init__("config_test", parameter_overrides=[])
        self.declare_parameter('test_param', 'Parameter not set')  
        self.printParam(None)
        self.add_on_set_parameters_callback(self.printParam)
        
    def printParam(self, new_params: Optional[List[Parameter]]):
        self.get_logger().info('test_param: %s' % self.get_parameter('test_param').value)
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = ConfigTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()