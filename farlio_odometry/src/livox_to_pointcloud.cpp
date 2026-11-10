// Licensed under the Apache License, Version 2.0.
//
// Livox CustomMsg -> sensor_msgs/PointCloud2 adapter for FAR-LIO.
//
// FAR-LIO consumes PointCloud2, while the Livox ROS 1 driver (and the CICV/AVIA
// bags) publish livox_ros_driver/CustomMsg. The converter keeps the Livox
// per-point nanosecond offset in an "offset_time" (uint32) field, which the
// FAR-LIO ROS 1 conversions understand as a per-point timestamp, so motion
// de-skewing works.
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <livox_ros_driver/CustomMsg.h>

#include <cmath>
#include <string>

namespace farlio_ros1 {

class LivoxToPointCloud {
 public:
  LivoxToPointCloud() : nh_(), pnh_("~") {
    pnh_.param<std::string>("input_topic", input_topic_, "/livox/lidar");
    pnh_.param<std::string>("output_topic", output_topic_, "/livox/lidar_points");
    pnh_.param<bool>("publish_line", publish_line_, true);
    pnh_.param<bool>("auto_stamp_offset", auto_stamp_offset_, false);
    pnh_.param<double>("stamp_offset", stamp_offset_, 0.0);
    pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 5);
    sub_ = nh_.subscribe<livox_ros_driver::CustomMsg>(
      input_topic_, 5, &LivoxToPointCloud::callback, this);
    ROS_INFO(
      "[livox_to_pointcloud] %s (livox_ros_driver/CustomMsg) -> %s (sensor_msgs/PointCloud2)",
      input_topic_.c_str(), output_topic_.c_str());
  }

 private:
  void callback(const livox_ros_driver::CustomMsg::ConstPtr& msg) {
    if (auto_stamp_offset_ && !offset_initialized_) {
      // Bags recorded from a driver replay can carry old message stamps while the
      // bag clock is "now". Align the first stamp with the current ROS time.
      stamp_offset_ = (ros::Time::now() - msg->header.stamp).toSec();
      offset_initialized_ = true;
      ROS_INFO("[livox_to_pointcloud] auto stamp offset %.3f s", stamp_offset_);
    }
    sensor_msgs::PointCloud2 out;
    out.header = msg->header;
    if (out.header.stamp.isZero() && msg->timebase > 0) {
      out.header.stamp.fromNSec(msg->timebase);
    }
    if (stamp_offset_ != 0.0) out.header.stamp += ros::Duration(stamp_offset_);
    out.height = 1;
    out.width = msg->point_num;
    out.is_bigendian = false;
    out.is_dense = false;
    sensor_msgs::PointCloud2Modifier modifier(out);
    if (publish_line_) {
      modifier.setPointCloud2Fields(
        6, "x", 1, sensor_msgs::PointField::FLOAT32, "y", 1,
        sensor_msgs::PointField::FLOAT32, "z", 1, sensor_msgs::PointField::FLOAT32, "intensity", 1,
        sensor_msgs::PointField::FLOAT32, "offset_time", 1, sensor_msgs::PointField::UINT32, "line",
        1, sensor_msgs::PointField::UINT8);
    } else {
      modifier.setPointCloud2Fields(
        5, "x", 1, sensor_msgs::PointField::FLOAT32, "y", 1,
        sensor_msgs::PointField::FLOAT32, "z", 1, sensor_msgs::PointField::FLOAT32, "intensity", 1,
        sensor_msgs::PointField::FLOAT32, "offset_time", 1, sensor_msgs::PointField::UINT32);
    }
    modifier.resize(msg->point_num);

    sensor_msgs::PointCloud2Iterator<float> ix(out, "x");
    sensor_msgs::PointCloud2Iterator<float> iy(out, "y");
    sensor_msgs::PointCloud2Iterator<float> iz(out, "z");
    sensor_msgs::PointCloud2Iterator<float> ii(out, "intensity");
    sensor_msgs::PointCloud2Iterator<uint32_t> it(out, "offset_time");
    std::unique_ptr<sensor_msgs::PointCloud2Iterator<uint8_t>> il;
    if (publish_line_) il.reset(new sensor_msgs::PointCloud2Iterator<uint8_t>(out, "line"));

    for (const auto& p : msg->points) {
      *ix = p.x;
      *iy = p.y;
      *iz = p.z;
      *ii = static_cast<float>(p.reflectivity);
      *it = p.offset_time;
      if (il) *(*il) = p.line;
      ++ix;
      ++iy;
      ++iz;
      ++ii;
      ++it;
      if (il) ++(*il);
    }
    pub_.publish(out);
    ROS_INFO_THROTTLE(
      10.0, "[livox_to_pointcloud] %u points, frame %s", msg->point_num,
      out.header.frame_id.c_str());
  }

  ros::NodeHandle nh_, pnh_;
  std::string input_topic_, output_topic_;
  bool publish_line_{true};
  bool auto_stamp_offset_{false};
  bool offset_initialized_{false};
  double stamp_offset_{0.0};
  ros::Subscriber sub_;
  ros::Publisher pub_;
};

}  // namespace farlio_ros1

int main(int argc, char** argv) {
  ros::init(argc, argv, "livox_to_pointcloud");
  farlio_ros1::LivoxToPointCloud node;
  ros::spin();
  return 0;
}
