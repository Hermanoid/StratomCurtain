#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/concave_hull.h>

class BlobExtractionNode : public rclcpp::Node{

public:

  BlobExtractionNode() : Node("blob_extraction_node"){

    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>("point_cloud", 10, std::bind(&BlobExtractionNode::pointCloudCallback, this, std::placeholders::_1));
    polygon_pub_ = create_publisher<geometry_msgs::msg::PolygonStamped>("cluster_polygon", 10);
    declare_parameter<int>("convex_alpha", 0.5);
    declare_parameter<int>("polygon_select", 0);
  }

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){

    pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *pclCloud);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(pclCloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclideanCluster;
    euclideanCluster.setClusterTolerance(0.2); // Adjust the cluster tolerance as needed
    euclideanCluster.setMinClusterSize(5);   // Adjust the minimum cluster size as needed
    euclideanCluster.setMaxClusterSize(10000); // Adjust the maximum cluster size as needed
    euclideanCluster.setSearchMethod(tree);
    euclideanCluster.setInputCloud(pclCloud);
    euclideanCluster.extract(clusterIndices);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConcaveHull<pcl::PointXYZ> chull;
    double alpha = get_parameter("convex_alpha").as_double();
    chull.setAlpha (alpha);
    size_t polygon_select = (int)get_parameter("polygon_select").as_int();
    // Process each cluster
    for (size_t i = 0; i < clusterIndices.size() && i < maxClusters_; ++i)
    {
      cluster.reset(new pcl::PointCloud<pcl::PointXYZ>);
      for (const auto& index : clusterIndices[i].indices)
      {
        cluster->push_back(pclCloud->points[index]);
      }

      chull.setInputCloud (cluster);
      chull.reconstruct (*cloud_hull);

      geometry_msgs::msg::Polygon polygon;
      polygon.points = std::vector<geometry_msgs::msg::Point32>();
      for(const auto& point : cloud_hull->points){
        geometry_msgs::msg::Point32 p;
        p.x = point.x;
        p.y = point.y;
        p.z = point.z;
        polygon.points.push_back(p);
      }

      geometry_msgs::msg::PolygonStamped polygonStamped;
      polygonStamped.header = msg->header;
      polygonStamped.polygon = polygon;

      if(polygon_select == i){
      polygon_pub_->publish(polygonStamped);

      }
    }
  }
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr polygon_pub_;
  const int maxClusters_ = 10; // Maximum number of clusters to publish
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BlobExtractionNode>());
  rclcpp::shutdown();
  
  return 0;
}
