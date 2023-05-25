import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2 as pc2
from sensor_msgs.msg import LaserScan
from laser_geometry import LaserProjection

class Laser2PC():
    def __init__(self):
        self.laserProj = LaserProjection()
        self.pcPub = self.create_publisher("/laserPointCloud", pc2, queue_size = 1)
        self.laserSub = self.create_subscription("/scan", LaserScan, self.laserCallback)

        def laserCallback(self, data):

            cloud_out = self.laserProj.projectLaser(data)

            self.pcPub.publish(cloud_out)
        
if __name__ == '__main__':
    rclpy.init(args=args)
    l2pc = Laser2PC()
    rclpy.spin(l2pc)
