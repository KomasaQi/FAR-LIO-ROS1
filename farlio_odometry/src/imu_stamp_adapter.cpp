// Licensed under the Apache License, Version 2.0.
//
// IMU stamp adapter for bags whose message timestamps do not match the ROS clock.
// The AVIA/CICV bag carries stamps from 2020 while `rosbag play --clock` publishes
// "now"; feeding the old stamps directly into the EKF makes the delay
// compensation reject every measurement. This node republishes the IMU stream
// with a constant offset, computed from the first message when
// `auto_stamp_offset` is true.
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <string>

namespace farlio_ros1 {

class ImuStampAdapter {
 public:
  ImuStampAdapter() : nh_(), pnh_("~") {
    pnh_.param<std::string>("input_topic", input_topic_, "/livox/imu");
    pnh_.param<std::string>("output_topic", output_topic_, "/livox/imu_fixed");
    pnh_.param<bool>("auto_stamp_offset", auto_stamp_offset_, false);
    pnh_.param<double>("stamp_offset", stamp_offset_, 0.0);
    pub_ = nh_.advertise<sensor_msgs::Imu>(output_topic_, 50);
    sub_ = nh_.subscribe<sensor_msgs::Imu>(input_topic_, 50, &ImuStampAdapter::callback, this);
    ROS_INFO("[imu_stamp_adapter] %s -> %s", input_topic_.c_str(), output_topic_.c_str());
  }

 private:
  void callback(const sensor_msgs::Imu::ConstPtr& msg) {
    if (auto_stamp_offset_ && !offset_initialized_) {
      stamp_offset_ = (ros::Time::now() - msg->header.stamp).toSec();
      offset_initialized_ = true;
      ROS_INFO("[imu_stamp_adapter] auto stamp offset %.3f s", stamp_offset_);
    }
    sensor_msgs::Imu out = *msg;
    if (stamp_offset_ != 0.0) out.header.stamp += ros::Duration(stamp_offset_);
    pub_.publish(out);
  }

  ros::NodeHandle nh_, pnh_;
  std::string input_topic_, output_topic_;
  bool auto_stamp_offset_{false};
  bool offset_initialized_{false};
  double stamp_offset_{0.0};
  ros::Subscriber sub_;
  ros::Publisher pub_;
};

}  // namespace farlio_ros1

int main(int argc, char** argv) {
  ros::init(argc, argv, "imu_stamp_adapter");
  farlio_ros1::ImuStampAdapter node;
  ros::spin();
  return 0;
}
