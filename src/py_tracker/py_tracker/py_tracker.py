import rclpy
from rclpy.node import Node
from rclpy.time import Duration
from rclpy.clock import JumpThreshold, TimeJump
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from shapely import Polygon, Point, LineString
import shapely.ops
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from typing import List

# Import the Polygon message type as PolygonMsg because we already have the shapely.Polygon name taken
from geometry_msgs.msg import Polygon as PolygonMsg
from geometry_msgs.msg import Point32, PolygonStamped
from costmap_converter_msgs.msg import ObstacleArrayMsg, ObstacleMsg
from visualization_msgs.msg import MarkerArray
from py_tracker_msgs.msg import PyTrackerArrayMsg
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Header

# The . indicates a relative import. To debug with vscode, you'll need to
# run with "module": "py_tracker.py_tracker" in .vscode/launch.json
# (The usual "program" option doesn't launch in the same way as ros2 run,
# ... so the imports work differently)
from .VizUtils import visualize_tracks
from .Tracker import Tracker
from .MessageUtils import create_warnings_msg, create_marker_msg


class PyTracker(Node):
    def __init__(self):
        super().__init__("py_tracker", parameter_overrides=[], allow_undeclared_parameters=True)
        # ID to assign to the object, dictionary to keep track of mapped objects ID to centroid, number of consecutive frames marked dissapeeared
        self.nextObjectID = 1
        # When a very small object is detected, its polygon is expanded by this amount in all directions
        self.declare_parameter("small_object_buffer", 0.2)
        # ROS2 doesn't allow any complex types, including a nested array like [[pt1.x, pt1.y], [pt2.x, pt2.y], ...]
        # so for this polygon we have to get creative with a 1d array formatted [pt1.x, pt1.y, pt2.x, pt2.y, ...]
        self.declare_parameter("curtain_boundary", [0.0, 10.0, -10.0, -10.0, 10.0, -10.0])

        self.declare_parameter("obstacles_sub_topic", "costmap_obstacles")
        self.declare_parameter("marker_pub_topic", "dynamic_marker")
        self.declare_parameter("marker_pub_enabled", True)
        self.declare_parameter("curtain_pub_topic", "curtain")
        self.declare_parameter("curtain_pub_enabled", True)
        self.declare_parameter("curtain_pub_period", 1.0)
        self.declare_parameter("warnings_pub_topic", "warnings")
        self.declare_parameter("warnings_pub_enabled", True)
        self.declare_parameter("viz_enabled", False)

        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("map_frame", "map")

        self.poly_timer = None
        self.curtain_pub = None
        self.marker_pub = None
        self.warnings_pub = None
        self.obstacles_sub = None

        self.tracker = Tracker(self)

        self.update_parameters(None)
        self.add_on_set_parameters_callback(self.update_parameters)

        # Setup for finding robot position
        self.tf_buffer = Buffer(cache_time=Duration(seconds=15))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        def time_jump_callback(time_jump: TimeJump):
            self.get_logger().warning("Time jump/change detected! Clearing tf buffer.")
            self.tf_buffer.clear()

        threshold = JumpThreshold(min_forward=Duration(seconds=1), min_backward=Duration(seconds=-1), on_clock_change=True)
        self.jump_handle = self.get_clock().create_jump_callback(threshold, post_callback=time_jump_callback)

        # Debuggery Stuff
        self.get_logger().info("py_tracker ready!")

    def update_parameters(self, _):
        self.small_object_buffer = self.get_parameter("small_object_buffer").get_parameter_value().double_value
        self.base_frame = self.get_parameter("base_frame").get_parameter_value().string_value
        self.map_frame = self.get_parameter("map_frame").get_parameter_value().string_value

        boundary_array = self.get_parameter("curtain_boundary").get_parameter_value().double_array_value
        # This big-brain one-liner grabs every other point for x, every other point (shifted one) for y, and pairs them together
        boundary_points = list(zip(boundary_array[::2], boundary_array[1::2]))
        self.curtain_boundary = Polygon(boundary_points)
        polygon_msg = PolygonMsg(points=[Point32(x=x, y=y) for x, y in boundary_points])
        self.curtain_msg = PolygonStamped(polygon=polygon_msg, header=Header(frame_id=self.base_frame))

        self.tracker.update_parameters(self)

        # Grab the updated curtain publish rate, destroy the old timer, and create a new one with the new rate.
        # ROS does not allow changing the period of an preexisting timer.
        pub_rate = self.get_parameter("curtain_pub_period").get_parameter_value().double_value
        if not self.poly_timer or pub_rate != self.poly_timer.timer_period_ns / 1000000000.0:
            if self.poly_timer:
                self.poly_timer.destroy()
            self.poly_timer = self.create_timer(pub_rate, self.publish_curtain)

        self.curtain_pub = self.update_publisher(self.curtain_pub, PolygonStamped, "curtain_pub_topic", 1)
        self.marker_pub = self.update_publisher(self.marker_pub, MarkerArray, "marker_pub_topic", 10)
        self.warnings_pub = self.update_publisher(self.warnings_pub, PyTrackerArrayMsg, "warnings_pub_topic", 10)
        self.curtain_pub_enabled = self.get_parameter("curtain_pub_enabled").get_parameter_value().bool_value
        self.marker_pub_enabled = self.get_parameter("marker_pub_enabled").get_parameter_value().bool_value
        self.warnings_pub_enabled = self.get_parameter("warnings_pub_enabled").get_parameter_value().bool_value
        self.viz_enabled = self.get_parameter("viz_enabled").get_parameter_value().bool_value

        obstacles_sub_topic = self.get_parameter("obstacles_sub_topic").get_parameter_value().string_value
        if not self.obstacles_sub or obstacles_sub_topic != self.obstacles_sub.topic:
            if self.obstacles_sub:
                self.obstacles_sub.destroy()
            self.obstacles_sub = self.create_subscription(ObstacleArrayMsg, obstacles_sub_topic, self.listener_callback, 10)
        return SetParametersResult(successful=True)

    def update_publisher(self, publisher, type, parameter, qos):
        """
        Similar to the timer object, ROS does not allow changing the topic of publishers on existinig publisher objects
        Thus, we need to destroy the old object and create it anew
        """
        new_topic_name = self.get_parameter(parameter).get_parameter_value().string_value
        if not publisher or new_topic_name != publisher.topic:
            if publisher:
                publisher.destroy()
            return self.create_publisher(type, new_topic_name, qos)
        else:
            return publisher

    def publish_curtain(self):
        polygon_msg = self.curtain_msg
        polygon_msg.header.stamp = self.get_clock().now().to_msg()
        self.curtain_pub.publish(polygon_msg)

    def listener_callback(self, msg: ObstacleArrayMsg):
        polygons, bad_points = self.clean_polygons(list(msg.obstacles))

        self.tracker.update(polygons, msg.header)

        if self.marker_pub_enabled:
            marker_array = create_marker_msg(self.tracker.objects, msg.header)
            self.marker_pub.publish(marker_array)
        if self.viz_enabled:
            visualize_tracks(self.tracker.objects, polygons, bad_points)
        # Produce warnings last because the tf buffer tends to be a little out-of-date,
        # and the function call often needs to delay for a bit to get the robot transform.
        if self.warnings_pub_enabled:
            self.curtain_msg.header.stamp = msg.header.stamp

            async def pub_warnings():
                warnings_array = await create_warnings_msg(
                    self.tracker.objects, self.tf_buffer, self.map_frame, self.base_frame, self.curtain_msg
                )
                if warnings_array is not None:
                    self.warnings_pub.publish(warnings_array)

            self.executor.create_task(pub_warnings())

    def clean_polygons(self, obsArr: List[ObstacleMsg]):
        """Gets Shapely polygons from the polygons in the input message and filters bad polygons"""
        polygons = []
        small_objects = []
        bad_points = []
        for obstacle in obsArr:
            points = [(point.x, point.y) for point in obstacle.polygon.points]
            # Small objects will often not be reported as completed (3-plus point) polygons,
            # but they should still be tracked. So, we buffer them to make them into polygons.
            # Then we merge those small buffered polygons, because they often come in clumps.
            if len(points) < 3:
                bad_points += points
                if len(points) == 1:
                    polygon = Point(points[0]).buffer(self.small_object_buffer)
                else:
                    polygon = LineString(points).buffer(self.small_object_buffer)
                small_objects.append(polygon)
            else:
                polygon = Polygon(shell=points)
                polygons.append(polygon)

        # Simplify the bad geometries, because they will often have overlap
        merged = shapely.ops.unary_union(small_objects)
        # unary_union will return a polygon if the input can be merged to a single polygon,
        # or a MultiPolygon if the small objects are not connected.
        if isinstance(merged, Polygon):
            polygons.append(merged)
        else:
            polygons += list(merged.geoms)

        return polygons, bad_points


def main(args=None):
    rclpy.init(args=args)
    try:
        # # Run with 3 threads because the tf buffer can need to block for some time
        # executor = MultiThreadedExecutor(num_threads=3)
        # py_tracker_node = PyTracker()
        # executor.spin(py_tracker_node)
        # executor.shutdown()
        py_tracker_node = PyTracker()
        rclpy.spin(py_tracker_node)
        rclpy.shutdown()
    except (KeyboardInterrupt, ExternalShutdownException):
        print("PyTracker was terminated by the user.")


if __name__ == "__main__":
    main()
