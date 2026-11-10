// Licensed under the Apache License, Version 2.0.
//
// Minimal ROS 1 replacement for the TUM map loader used by FAR-LIO's
// localization mode:
//
//   farlio_map_server  --(farlio_msgs/GetPointCloudMap)-->  farlio_odometry_node
//
// It loads a PCD once, optionally voxel-downsamples it to the requested
// resolution and returns it as sensor_msgs/PointCloud2. The odometry node can
// then be configured with node.input_map: "/core/map/get_pointcloud_map"
// (service mode) exactly like upstream's tum_map_msgs workflow, including the
// asynchronous map-update service ("<service>_ready" trigger topic).
#include <ros/ros.h>

#include <farlio_msgs/GetPointCloudMap.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace farlio_ros1 {

class MapServer {
 public:
  MapServer() : nh_(), pnh_("~") {
    pnh_.param<std::string>("map_path", map_path_, "");
    pnh_.param<std::string>("frame_id", frame_id_, "local_cartesian");
    pnh_.param<std::string>("service_name", service_name_, "/core/map/get_pointcloud_map");
    if (map_path_.empty()) {
      ROS_FATAL("[farlio_map_server] ~map_path is required.");
      ros::shutdown();
      return;
    }
    if (!load()) {
      ros::shutdown();
      return;
    }
    srv_ = nh_.advertiseService(
      service_name_, &MapServer::callback, this);
    ROS_INFO(
      "[farlio_map_server] serving %zu points from %s on %s (frame %s)", cloud_.size(),
      map_path_.c_str(), service_name_.c_str(), frame_id_.c_str());
  }

 private:
  bool load() {
    pcl::PointCloud<pcl::PointXYZI> tmp;
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(map_path_, tmp) != 0) {
      ROS_FATAL("[farlio_map_server] cannot read %s", map_path_.c_str());
      return false;
    }
    for (const auto& p : tmp.points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
      pcl::PointXYZI pt;
      pt.x = p.x;
      pt.y = p.y;
      pt.z = p.z;
      pt.intensity = p.intensity;
      cloud_.push_back(pt);
    }
    return true;
  }

  // First-point-per-voxel downsampling (no PCL filters dependency).
  std::vector<pcl::PointXYZI> downsample(float voxel_size) const {
    if (voxel_size <= 0.0f) return cloud_;
    const float inv = 1.0f / voxel_size;
    std::unordered_map<std::int64_t, std::size_t> seen;
    seen.reserve(cloud_.size());
    std::vector<pcl::PointXYZI> out;
    out.reserve(cloud_.size());
    for (const auto& p : cloud_) {
      const std::int64_t ix = static_cast<std::int64_t>(std::floor(p.x * inv));
      const std::int64_t iy = static_cast<std::int64_t>(std::floor(p.y * inv));
      const std::int64_t iz = static_cast<std::int64_t>(std::floor(p.z * inv));
      const std::int64_t key = (ix * 73856093LL) ^ (iy * 19349663LL) ^ (iz * 83492791LL);
      if (seen.emplace(key, out.size()).second) out.push_back(p);
    }
    return out;
  }

  bool callback(farlio_msgs::GetPointCloudMap::Request& req,
                farlio_msgs::GetPointCloudMap::Response& res) {
    const auto points = downsample(req.voxel_size);
    sensor_msgs::PointCloud2 msg;
    msg.header.stamp = req.header.stamp.isZero() ? ros::Time::now() : req.header.stamp;
    msg.header.frame_id = req.header.frame_id.empty() ? frame_id_ : req.header.frame_id;
    msg.height = 1;
    msg.width = static_cast<uint32_t>(points.size());
    msg.is_bigendian = false;
    msg.is_dense = true;
    sensor_msgs::PointCloud2Modifier modifier(msg);
    modifier.setPointCloud2Fields(
      4, "x", 1, sensor_msgs::PointField::FLOAT32, "y", 1, sensor_msgs::PointField::FLOAT32,
      "z", 1, sensor_msgs::PointField::FLOAT32, "intensity", 1, sensor_msgs::PointField::FLOAT32);
    modifier.resize(points.size());
    sensor_msgs::PointCloud2Iterator<float> ix(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iy(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iz(msg, "z");
    sensor_msgs::PointCloud2Iterator<float> ii(msg, "intensity");
    for (const auto& p : points) {
      *ix = p.x;
      *iy = p.y;
      *iz = p.z;
      *ii = p.intensity;
      ++ix;
      ++iy;
      ++iz;
      ++ii;
    }
    res.map = msg;
    res.success = true;
    ROS_INFO_THROTTLE(
      5.0, "[farlio_map_server] served %zu points (voxel %.2f m)", points.size(), req.voxel_size);
    return true;
  }

  ros::NodeHandle nh_, pnh_;
  std::string map_path_, frame_id_, service_name_;
  std::vector<pcl::PointXYZI> cloud_;
  ros::ServiceServer srv_;
};

}  // namespace farlio_ros1

int main(int argc, char** argv) {
  ros::init(argc, argv, "farlio_map_server");
  farlio_ros1::MapServer server;
  ros::spin();
  return 0;
}
