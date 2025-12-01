#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <pcl/search/impl/kdtree.hpp>
#include <vector>

using namespace std;
using namespace Eigen;

// ROS2 Initialize Node
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud;

sensor_msgs::msg::PointCloud2 local_map_pcl;
sensor_msgs::msg::PointCloud2 local_depth_pcl;


rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_sub;
rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr local_map_sub;

rclcpp::TimerBase::SharedPtr local_sensing_timer;


bool has_global_map = false;
bool has_local_map = false;
bool has_odom = false;

nav_msgs::msg::Odometry _odom;

double sensing_horizon, sensing_rate, estimation_rate;
double _x_size, _y_size, _z_size;
double _gl_xl, _gl_yl, _gl_zl;
double _resolution, _inv_resolution;
int _GLX_SIZE, _GLY_SIZE, _GLZ_SIZE;

rclcpp::Time last_odom_stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);

inline Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i& index) {
  Eigen::Vector3d pt;
  pt(0) = ((double)index(0) + 0.5) * _resolution + _gl_xl;
  pt(1) = ((double)index(1) + 0.5) * _resolution + _gl_yl;
  pt(2) = ((double)index(2) + 0.5) * _resolution + _gl_zl;

  return pt;
};

inline Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d& pt) {
  Eigen::Vector3i idx;
  idx(0) = std::min(std::max(int((pt(0) - _gl_xl) * _inv_resolution), 0),
                    _GLX_SIZE - 1);
  idx(1) = std::min(std::max(int((pt(1) - _gl_yl) * _inv_resolution), 0),
                    _GLY_SIZE - 1);
  idx(2) = std::min(std::max(int((pt(2) - _gl_zl) * _inv_resolution), 0),
                    _GLZ_SIZE - 1);

  return idx;
};

// Odometer information callback
void rcvOdometryCallbck(const nav_msgs::msg::Odometry& odom) {
  /*if(!has_global_map)
    return;*/
  has_odom = true;
  _odom = odom;
}

pcl::PointCloud<pcl::PointXYZ> _cloud_all_map, _local_map;
pcl::VoxelGrid<pcl::PointXYZ> _voxel_sampler;
sensor_msgs::msg::PointCloud2 _local_map_pcd;

pcl::search::KdTree<pcl::PointXYZ> _kdtreeLocalMap;
vector<int> _pointIdxRadiusSearch;
vector<float> _pointRadiusSquaredDistance;

void rcvGlobalPointCloudCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_map) {
  if (has_global_map) return;

  RCLCPP_WARN(rclcpp::get_logger("rcvGlobalPointCloudCallBack"), "Global Pointcloud received..");

  // Convert message format
  pcl::PointCloud<pcl::PointXYZ> cloud_input;
  pcl::fromROSMsg(*pointcloud_map, cloud_input);

  // Use voxel filtering to downsample point clouds
  _voxel_sampler.setLeafSize(0.1f, 0.1f, 0.1f);
  _voxel_sampler.setInputCloud(cloud_input.makeShared());
  // The results are stored in _cloud_all_map
  _voxel_sampler.filter(_cloud_all_map);

  _kdtreeLocalMap.setInputCloud(_cloud_all_map.makeShared());

  has_global_map = true;
}

void renderSensedPoints(/*const rclcpp::TimerBase event*/) {
  if (!has_global_map || !has_odom) return;

  // Get the drone's attitude
  Eigen::Quaterniond q;
  q.x() = _odom.pose.pose.orientation.x;
  q.y() = _odom.pose.pose.orientation.y;
  q.z() = _odom.pose.pose.orientation.z;
  q.w() = _odom.pose.pose.orientation.w;

  Eigen::Matrix3d rot;
  rot = q;
  // Convert to rotation matrix
  Eigen::Vector3d yaw_vec = rot.col(0);

  // Clear and initialize point cloud data
  _local_map.points.clear();
  pcl::PointXYZ searchPoint(_odom.pose.pose.position.x,
                            _odom.pose.pose.position.y,
                            _odom.pose.pose.position.z);
  _pointIdxRadiusSearch.clear();
  _pointRadiusSquaredDistance.clear();

  pcl::PointXYZ pt;
  // Perform a radius search to obtain points within the perception range
  if (_kdtreeLocalMap.radiusSearch(searchPoint, sensing_horizon,
                                   _pointIdxRadiusSearch,
                                   _pointRadiusSquaredDistance) > 0) {
    // Iterate through each searched point
    for (size_t i = 0; i < _pointIdxRadiusSearch.size(); ++i) {
      pt = _cloud_all_map.points[_pointIdxRadiusSearch[i]];

      // Set maximum elevation angle
      if ((fabs(pt.z - _odom.pose.pose.position.z) / sensing_horizon) >
          tan(M_PI / 6.0))
        continue;

      // Check if the checkpoint is within the drone's field of view
      Eigen::Vector3d pt_vec(pt.x - _odom.pose.pose.position.x,
                             pt.y - _odom.pose.pose.position.y,
                             pt.z - _odom.pose.pose.position.z);

      if (pt_vec.normalized().dot(yaw_vec) < 0.5) continue;

      _local_map.points.push_back(pt);
    }
  } else {
    return;
  }

  // Set local map properties and publish
  _local_map.width = _local_map.points.size();
  _local_map.height = 1;
  _local_map.is_dense = true;

  pcl::toROSMsg(_local_map, _local_map_pcd);
  _local_map_pcd.header.frame_id = "map";

  pub_cloud->publish(_local_map_pcd);
}

void rcvLocalPointCloudCallBack(
    const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_map) {
  // do nothing, fix later
}

int main(int argc, char** argv) {
  // Initialize ROS2
  rclcpp::init(argc, argv);

  // Create node
  auto node = rclcpp::Node::make_shared("pcl_render");

  // Use declare_parameter to declare and read parameters.
  node->declare_parameter("sensing_horizon", 0.0);
  node->declare_parameter("sensing_rate", 0.0);
  node->declare_parameter("estimation_rate", 0.0);

  node->declare_parameter("map/x_size", 0.0);
  node->declare_parameter("map/y_size", 0.0);
  node->declare_parameter("map/z_size", 0.0);

  node->get_parameter("sensing_horizon", sensing_horizon);
  node->get_parameter("sensing_rate", sensing_rate);
  node->get_parameter("estimation_rate", estimation_rate);
  node->get_parameter("map/x_size", _x_size);
  node->get_parameter("map/y_size", _y_size);
  node->get_parameter("map/z_size", _z_size);

  // Subscribe to point cloud data
  global_map_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
      "global_map", 1, rcvGlobalPointCloudCallBack);
  local_map_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
      "local_map", 1, rcvLocalPointCloudCallBack);
  odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
      "/odometry", 50, rcvOdometryCallbck);

  // Published by: Point Cloud Data
  pub_cloud = node->create_publisher<sensor_msgs::msg::PointCloud2>("pcl_render_node/cloud", 10);

  // Timer: Controls rendering frequency
  double sensing_duration = 1.0 / sensing_rate * 2.5;
  local_sensing_timer = node->create_wall_timer(
      std::chrono::duration<double>(sensing_duration), std::bind(&renderSensedPoints));

  _inv_resolution = 1.0 / _resolution;

  _gl_xl = -_x_size / 2.0;
  _gl_yl = -_y_size / 2.0;
  _gl_zl = 0.0;

  _GLX_SIZE = static_cast<int>(_x_size * _inv_resolution);
  _GLY_SIZE = static_cast<int>(_y_size * _inv_resolution);
  _GLZ_SIZE = static_cast<int>(_z_size * _inv_resolution);

  rclcpp::Rate rate(100);
  bool status = rclcpp::ok();
  while (status) {
    rclcpp::spin_some(node);  
    status = rclcpp::ok();
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}