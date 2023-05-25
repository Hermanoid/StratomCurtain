#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>

class BlobExtractionNode : public rclcpp::Node{

public:

  BlobExtractionNode() : Node("blob_extraction_node"){

    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>("point_cloud", 10, std::bind(&BlobExtractionNode::pointCloudCallback, this, std::placeholders::_1));
    clusterPublishers_.resize(maxClusters_);
    for (int i = 0; i < maxClusters_; ++i){
      std::string topicName = "cluster_" + std::to_string(i);
      clusterPublishers_[i] = create_publisher<sensor_msgs::msg::PointCloud2>(topicName, 10);
    }
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

    // Process each cluster
    for (int i = 0; i < clusterIndices.size() && i < maxClusters_; ++i)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
      for (const auto& index : clusterIndices[i].indices)
      {
        cluster->push_back(pclCloud->points[index]);
      }

      // Convert the cluster to PointCloud2 message
      sensor_msgs::msg::PointCloud2::SharedPtr clusterMsg(new sensor_msgs::msg::PointCloud2);
      pcl::toROSMsg(*cluster, *clusterMsg);
      clusterMsg->header = msg->header;

      // Publish the cluster on a new topic
      clusterPublishers_[i]->publish(*clusterMsg);
    }
  }
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> clusterPublishers_;
  const int maxClusters_ = 10; // Maximum number of clusters to publish
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BlobExtractionNode>());
  rclcpp::shutdown();
  
  return 0;
}
