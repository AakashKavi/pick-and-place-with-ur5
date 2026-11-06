#include "ur5_pick_place_perception/object_detector.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/header.hpp>

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
ObjectDetectorNode::ObjectDetectorNode()
: Node("object_detector")
{
  // Declare parameters with defaults
  this->declare_parameter("cluster_tolerance",      0.02);
  this->declare_parameter("min_cluster_size",        50);
  this->declare_parameter("max_cluster_size",      25000);
  this->declare_parameter("passthrough_min_z",      0.55);
  this->declare_parameter("passthrough_max_z",      0.79);
  this->declare_parameter("voxel_leaf_size",        0.01);
  this->declare_parameter("plane_distance_threshold", 0.01);
  this->declare_parameter("enable_plane_removal", false);

  cluster_tolerance_        = this->get_parameter("cluster_tolerance").as_double();
  min_cluster_size_         = this->get_parameter("min_cluster_size").as_int();
  max_cluster_size_         = this->get_parameter("max_cluster_size").as_int();
  passthrough_min_z_        = this->get_parameter("passthrough_min_z").as_double();
  passthrough_max_z_        = this->get_parameter("passthrough_max_z").as_double();
  voxel_leaf_size_          = this->get_parameter("voxel_leaf_size").as_double();
  plane_distance_threshold_ = this->get_parameter("plane_distance_threshold").as_double();
  enable_plane_removal_     = this->get_parameter("enable_plane_removal").as_bool();

  // Subscriber
  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/camera/depth/points", rclcpp::SensorDataQoS(),
    std::bind(&ObjectDetectorNode::pointcloudCallback, this, std::placeholders::_1));

  // Publishers
  object_poses_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
    "/detected_objects", 10);

  markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/detected_objects_markers", 10);

  RCLCPP_INFO(this->get_logger(), "Object detector initialized. Listening on /camera/depth/points");
}

// ---------------------------------------------------------------------------
// Main callback — runs the full pipeline on each incoming cloud
// ---------------------------------------------------------------------------
void ObjectDetectorNode::pointcloudCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // Convert from ROS message to PCL type
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);

  if (cloud->empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Received empty point cloud");
    return;
  }

  // --- Pipeline ---
  // Plane removal is optional (enable_plane_removal, default false): the
  // passthrough crop below already excludes the table surface for this
  // specific camera/table geometry (table top is at 0.80 m depth from the
  // camera, the crop's upper bound is 0.79 m), so RANSAC plane removal is
  // redundant work here. It is kept available and wired in because that
  // depth margin is scene-specific and will not hold for a different
  // table height or camera mount.
  auto downsampled  = downsample(cloud);
  auto filtered     = passthroughFilter(downsampled);
  auto objects_only = enable_plane_removal_ ? removePlane(filtered) : filtered;
  auto clusters     = extractClusters(objects_only);

  // Build PoseArray from cluster centroids
  geometry_msgs::msg::PoseArray pose_array;
  pose_array.header = msg->header;

  for (const auto & cluster : clusters) {
    pose_array.poses.push_back(clusterToPose(cluster));
  }

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
    "Detected %zu object(s)", clusters.size());

  object_poses_pub_->publish(pose_array);
  publishMarkers(pose_array, msg->header);
}

// ---------------------------------------------------------------------------
// Stage 1: Voxel Grid downsampling
//
// Think of this like reducing the resolution of a photo before processing it —
// fewer points means faster computation, with minimal loss of shape information.
// ---------------------------------------------------------------------------
pcl::PointCloud<pcl::PointXYZ>::Ptr ObjectDetectorNode::downsample(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
{
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setInputCloud(cloud);
  float leaf = static_cast<float>(voxel_leaf_size_);
  voxel.setLeafSize(leaf, leaf, leaf);

  pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>);
  voxel.filter(*out);
  return out;
}

// ---------------------------------------------------------------------------
// Stage 2: PassThrough filter (crop to workspace)
//
// We only care about the region directly above the table. Anything too close
// to the camera or too far below is noise we can safely discard.
// ---------------------------------------------------------------------------
pcl::PointCloud<pcl::PointXYZ>::Ptr ObjectDetectorNode::passthroughFilter(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
{
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(passthrough_min_z_, passthrough_max_z_);

  pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>);
  pass.filter(*out);
  return out;
}

// ---------------------------------------------------------------------------
// Stage 3: RANSAC plane removal (remove the table surface)
//
// RANSAC randomly samples 3 points, fits a plane, and counts how many other
// points agree (within a threshold). It repeats this many times and keeps
// the plane that has the most agreement — that is the table. We then discard
// those inlier points, keeping only the objects sitting on top.
// ---------------------------------------------------------------------------
pcl::PointCloud<pcl::PointXYZ>::Ptr ObjectDetectorNode::removePlane(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
{
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(100);
  seg.setDistanceThreshold(plane_distance_threshold_);
  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.empty()) {
    RCLCPP_WARN(this->get_logger(), "No dominant plane found in point cloud");
    return cloud;  // Return unmodified if no plane found
  }

  // Remove the plane inliers, keep everything above it
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);  // true = keep the NON-plane points

  pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>);
  extract.filter(*out);
  return out;
}

// ---------------------------------------------------------------------------
// Stage 4: Euclidean Cluster Extraction
//
// Points that are close together (within cluster_tolerance) get grouped into
// one cluster — each cluster represents one object. This works like grouping
// islands on a map: if two pieces of land are close enough, they belong to
// the same island.
// ---------------------------------------------------------------------------
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> ObjectDetectorNode::extractClusters(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
{
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> result;

  if (cloud->empty()) return result;

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(cluster_tolerance_);
  ec.setMinClusterSize(min_cluster_size_);
  ec.setMaxClusterSize(max_cluster_size_);
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

// ---------------------------------------------------------------------------
// Compute centroid of a cluster and return as a Pose
// Orientation is fixed to a top-down grasp (identity quaternion).
// ---------------------------------------------------------------------------
geometry_msgs::msg::Pose ObjectDetectorNode::clusterToPose(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cluster)
{
  double cx = 0.0, cy = 0.0, cz = 0.0;
  for (const auto & point : cluster->points) {
    cx += point.x;
    cy += point.y;
    cz += point.z;
  }
  double n = static_cast<double>(cluster->size());

  geometry_msgs::msg::Pose pose;
  pose.position.x = cx / n;
  pose.position.y = cy / n;
  pose.position.z = cz / n;
  pose.orientation.w = 1.0;  // Top-down grasp, no rotation
  return pose;
}

// ---------------------------------------------------------------------------
// Publish sphere markers in RViz for each detected object
// ---------------------------------------------------------------------------
void ObjectDetectorNode::publishMarkers(
  const geometry_msgs::msg::PoseArray & poses,
  const std_msgs::msg::Header & header)
{
  visualization_msgs::msg::MarkerArray marker_array;

  // First add a DELETE_ALL marker to clear stale markers
  visualization_msgs::msg::Marker delete_marker;
  delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(delete_marker);

  for (size_t i = 0; i < poses.poses.size(); ++i) {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns = "detected_objects";
    marker.id = static_cast<int>(i);
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = poses.poses[i];
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.8f;
    marker_array.markers.push_back(marker);
  }

  markers_pub_->publish(marker_array);
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
