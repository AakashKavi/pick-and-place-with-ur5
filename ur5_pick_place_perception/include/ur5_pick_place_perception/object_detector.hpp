#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "ur5_pick_place_perception/clustering.hpp"

class ObjectDetectorNode : public rclcpp::Node
{
public:
  ObjectDetectorNode();

private:
  // Callback
  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  // Pipeline stages
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsample(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr passthroughFilter(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr removePlane(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud);

  geometry_msgs::msg::Pose clusterToPose(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cluster);

  void publishMarkers(
    const geometry_msgs::msg::PoseArray & poses,
    const std_msgs::msg::Header & header);

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr object_poses_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;

  // Parameters
  double cluster_tolerance_;
  int min_cluster_size_;
  int max_cluster_size_;
  double passthrough_min_;
  double passthrough_max_;
  std::string passthrough_axis_;
  double voxel_leaf_size_;
  double plane_distance_threshold_;
  bool enable_plane_removal_;
};
