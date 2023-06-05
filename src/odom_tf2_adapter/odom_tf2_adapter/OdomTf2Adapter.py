import rclpy
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from nav_msgs.msg import Odometry



class FramePublisher(Node):

    def __init__(self):
        super().__init__("odom_tf2_adapter", parameter_overrides=[])
        self.get_logger().info("Starting OdomTf2Adapter node...")

        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("parent_frame", "odom")

        self.odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        self.base_frame = self.get_parameter("base_frame").get_parameter_value().string_value
        self.parent_frame = self.get_parameter("parent_frame").get_parameter_value().string_value


        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to a turtle{1}{2}/pose topic and call handle_turtle_pose
        # callback function on each message
        self.create_subscription(
            Odometry,
            self.odom_topic,
            self.handle_odom_msg,
            1)

        self.get_logger().info(f"Publishing {self.odom_topic} topic as a tf transform from {self.parent_frame} to {self.base_frame}")

    def handle_odom_msg(self, msg: Odometry):
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        # Override the frame_id and child_frame_id with the configured values
        t.header = msg.header
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.base_frame

        # Copy over the position information
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        # Copy over the orientation information
        t.transform.rotation.x = msg.pose.pose.orientation.x
        t.transform.rotation.y = msg.pose.pose.orientation.y
        t.transform.rotation.z = msg.pose.pose.orientation.z
        t.transform.rotation.w = msg.pose.pose.orientation.w

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == "__main__":
    main()