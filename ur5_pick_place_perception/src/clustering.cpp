#include "ur5_pick_place_perception/clustering.hpp"

#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

namespace ur5_pick_place_perception
{

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> extractClusters(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  double cluster_tolerance,
  int min_cluster_size,
  int max_cluster_size)
{
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> result;

  if (cloud->empty()) return result;

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(cluster_tolerance);
  ec.setMinClusterSize(min_cluster_size);
  ec.setMaxClusterSize(max_cluster_size);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

  for (const auto & indices : cluster_indices) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
    for (int idx : indices.indices) {
      cluster->push_back(cloud->points[idx]);
    }
    result.push_back(cluster);
  }

  return result;
}

}  // namespace ur5_pick_place_perception
