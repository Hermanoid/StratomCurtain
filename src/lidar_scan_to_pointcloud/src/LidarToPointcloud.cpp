#include <rclcpp/rclcpp.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class LidarToPointCloudNode : public rclcpp::Node {
public:
  LidarToPointCloudNode(): Node("lidar_to_pointcloud_node"){

    // Subscribe to /scan topic
    subscription_ = create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&LidarToPointCloudNode::scanCallback, this, std::placeholders::_1));

    // Create the Point Cloud publisher
    pointCloudPublisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud", 10);
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg){

    // Create a Point Cloud made of points
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    // Set the size of the Point Cloud
    pointCloud->width = msg->ranges.size();
    pointCloud->height = 1;
    pointCloud->is_dense = false;
    pointCloud->points.resize(pointCloud->width * pointCloud->height);

    // Populate the Point Cloud with LiDAR data
    for (size_t i = 0; i < msg->ranges.size(); ++i){

      // Convert polar coordinates to Cartesian coordinates
      float range = msg->ranges[i];
      float angle = msg->angle_min + i * msg->angle_increment;

      pointCloud->points[i].x = range * cos(angle);
      pointCloud->points[i].y = range * sin(angle);
      pointCloud->points[i].z = 0.0; 
    }

    // Convert the Point Cloud to sensor_msgs::msg::PointCloud2
    sensor_msgs::msg::PointCloud2::SharedPtr cloudMsg(new sensor_msgs::msg::PointCloud2);
    pcl::toROSMsg(*pointCloud, *cloudMsg);
    cloudMsg->header.frame_id = "base_scan";  // Set the frame ID

    // Publish the Point Cloud
    pointCloudPublisher_->publish(*cloudMsg);

    // Print information
    //RCLCPP_INFO(this->get_logger(), "Received LiDAR scan. Converted to Point Cloud and published.");
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloudPublisher_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarToPointCloudNode>());
  rclcpp::shutdown();

  return 0;
}
