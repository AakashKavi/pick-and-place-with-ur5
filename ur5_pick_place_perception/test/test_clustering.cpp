#include <gtest/gtest.h>

#include "ur5_pick_place_perception/clustering.hpp"

namespace
{

// A small grid of points, centered at (cx, cy, cz), spacing 0.02 m between
// neighbors in every axis — densely packed enough that EuclideanCluster
// Extraction's own distance threshold (cluster_tolerance, set below the
// inter-box distance but well above 0.02 m in every test here) keeps all
// of these points in the same cluster.
pcl::PointCloud<pcl::PointXYZ>::Ptr makeBox(float cx, float cy, float cz)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr box(new pcl::PointCloud<pcl::PointXYZ>);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 3; ++k) {
        box->push_back(pcl::PointXYZ(
          cx + 0.02f * i, cy + 0.02f * j, cz + 0.02f * k));
      }
    }
  }
  return box;
}

}  // namespace

TEST(ExtractClusters, TwoSeparatedBoxesGiveTwoClusters)
{
  // Two 3x3x3 point grids (27 points each), one centered near the origin
  // and one offset by 1.0 m in x — far beyond any reasonable
  // cluster_tolerance, so EuclideanClusterExtraction must not merge them.
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  auto box_a = makeBox(0.0f, 0.0f, 0.0f);
  auto box_b = makeBox(1.0f, 0.0f, 0.0f);
  *cloud += *box_a;
  *cloud += *box_b;

  auto clusters = ur5_pick_place_perception::extractClusters(
    cloud, /*cluster_tolerance=*/0.05, /*min_cluster_size=*/5, /*max_cluster_size=*/1000);

  ASSERT_EQ(clusters.size(), 2u);
  // Order between the two clusters is not guaranteed by
  // EuclideanClusterExtraction, so check sizes as a set, not positionally.
  EXPECT_EQ(clusters[0]->size() + clusters[1]->size(), 27u + 27u);
  EXPECT_EQ(clusters[0]->size(), 27u);
  EXPECT_EQ(clusters[1]->size(), 27u);
}

TEST(ExtractClusters, OneBoxGivesOneCluster)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  *cloud += *makeBox(0.0f, 0.0f, 0.0f);

  auto clusters = ur5_pick_place_perception::extractClusters(
    cloud, /*cluster_tolerance=*/0.05, /*min_cluster_size=*/5, /*max_cluster_size=*/1000);

  ASSERT_EQ(clusters.size(), 1u);
  EXPECT_EQ(clusters[0]->size(), 27u);
}

TEST(ExtractClusters, EmptyCloudGivesNoClusters)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  auto clusters = ur5_pick_place_perception::extractClusters(
    cloud, /*cluster_tolerance=*/0.05, /*min_cluster_size=*/5, /*max_cluster_size=*/1000);
  EXPECT_TRUE(clusters.empty());
}

TEST(ExtractClusters, BelowMinClusterSizeIsDropped)
{
  // A single isolated point can never satisfy min_cluster_size=5, so it
  // should be dropped entirely rather than reported as a 1-point cluster.
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->push_back(pcl::PointXYZ(0.0f, 0.0f, 0.0f));

  auto clusters = ur5_pick_place_perception::extractClusters(
    cloud, /*cluster_tolerance=*/0.05, /*min_cluster_size=*/5, /*max_cluster_size=*/1000);

  EXPECT_TRUE(clusters.empty());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
