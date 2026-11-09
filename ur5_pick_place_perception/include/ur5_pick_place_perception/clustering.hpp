#pragma once

#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace ur5_pick_place_perception
{

// Euclidean cluster extraction: points within cluster_tolerance of each
// other are grouped into one cluster, each representing one object.
//
// Pulled out of ObjectDetectorNode into a free function, taking the
// three parameters it actually needs explicitly instead of reading them
// off node member state, specifically so it can be unit tested on a
// synthetic point cloud without constructing a full rclcpp node (see
// test/test_clustering.cpp). Behavior is unchanged from when this lived
// as ObjectDetectorNode::extractClusters; see object_detector.cpp's
// pointcloudCallback for the full pipeline this is stage 4 of.
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> extractClusters(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  double cluster_tolerance,
  int min_cluster_size,
  int max_cluster_size);

}  // namespace ur5_pick_place_perception
