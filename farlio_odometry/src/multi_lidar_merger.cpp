// Licensed under the Apache License, Version 2.0.
//
// Multi-LiDAR front-end for FAR-LIO.
//
// FAR-LIO registers exactly one point cloud per frame. To use several LiDARs
// (e.g. Hesai 64 + Livox Avia/Mid360/Airy) they have to be merged into one cloud
// expressed in a common vehicle frame, with a per-point timestamp relative to the
// merged frame stamp. This node does exactly that:
//
//   /lidar_front  ─┐
//   /lidar_left   ─┼─> transform to target_frame, concatenate, add "time" field
//   /lidar_right  ─┘                 └─> /merged_points ─> farlio_odometry_node
//
// Clouds are grouped by arrival: whenever one cloud per topic is buffered and the
// spread of their stamps is <= max_stamp_delta, they are merged and published.
// Hardware time synchronisation (PTP/PPS) between the sensors is strongly
// recommended; otherwise the point times of the different sensors are not
// consistent and the de-skewing will be wrong.
#include <ros/ros.h>

#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <cstdint>
#include <map>
#include <sstream>
#include <mutex>
#include <optional>
#include <string>
#include <type_traits>
#include <variant>
#include <vector>

namespace farlio_ros1 {

class MultiLidarMerger {
 public:
  MultiLidarMerger() : nh_(), pnh_("~") {
    pnh_.param<std::string>("output_topic", output_topic_, "/merged_points");
    pnh_.param<std::string>("output_frame", output_frame_, "base_link");
    pnh_.param<double>("max_stamp_delta", max_stamp_delta_, 0.02);
    pnh_.param<bool>("publish_intensity", publish_intensity_, true);
    pnh_.param<int>("queue_size", queue_size_, 5);
    pnh_.param<double>("tf_timeout", tf_timeout_, 0.05);

    std::vector<std::string> topics;
    if (!pnh_.getParam("input_topics", topics) || topics.empty()) {
      // Also accept a comma separated string, e.g. _input_topics:=/a,/b
      std::string csv;
      if (pnh_.getParam("input_topics", csv) && !csv.empty()) {
        std::stringstream ss(csv);
        std::string item;
        while (std::getline(ss, item, ',')) {
          if (!item.empty()) topics.push_back(item);
        }
      }
    }
    if (topics.empty()) {
      ROS_FATAL("[farlio_multi_lidar] ~input_topics is required (list of PointCloud2 topics).");
      ros::shutdown();
      return;
    }
    if (topics.size() < 2) {
      ROS_WARN("[farlio_multi_lidar] Only one input topic configured; the merger is pointless.");
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>();
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 2);

    for (const auto& topic : topics) {
      topics_.push_back(topic);
      subs_.push_back(nh_.subscribe<sensor_msgs::PointCloud2>(
        topic, queue_size_, [this, topic](const sensor_msgs::PointCloud2ConstPtr& msg) {
          cloud_callback(topic, msg);
        }));
      ROS_INFO("[farlio_multi_lidar] input: %s", topic.c_str());
    }
    ROS_INFO(
      "[farlio_multi_lidar] merging %zu topics -> %s (frame %s)", topics_.size(),
      output_topic_.c_str(), output_frame_.c_str());
  }

 private:
  struct Buffered {
    sensor_msgs::PointCloud2ConstPtr msg;
    std::uint64_t stamp_ns{0};
  };

  static std::uint64_t stamp2ns(const ros::Time& t) {
    return static_cast<std::uint64_t>(t.sec) * 1000000000ULL + static_cast<std::uint64_t>(t.nsec);
  }

  void cloud_callback(const std::string& topic, const sensor_msgs::PointCloud2ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    buffer_[topic] = Buffered{msg, stamp2ns(msg->header.stamp)};
    if (buffer_.size() < topics_.size()) return;

    std::uint64_t newest = 0, oldest = UINT64_MAX;
    for (const auto& kv : buffer_) {
      newest = std::max(newest, kv.second.stamp_ns);
      oldest = std::min(oldest, kv.second.stamp_ns);
    }
    if (static_cast<double>(newest - oldest) * 1e-9 > max_stamp_delta_) {
      // Not synchronised: drop the oldest entry so the buffers can re-converge.
      std::string oldest_topic;
      std::uint64_t oldest_stamp = UINT64_MAX;
      for (const auto& kv : buffer_) {
        if (kv.second.stamp_ns < oldest_stamp) {
          oldest_stamp = kv.second.stamp_ns;
          oldest_topic = kv.first;
        }
      }
      if (!oldest_topic.empty()) buffer_.erase(oldest_topic);
      return;
    }

    std::vector<Buffered> clouds;
    clouds.reserve(buffer_.size());
    for (const auto& kv : buffer_) clouds.push_back(kv.second);
    buffer_.clear();
    merge_and_publish(clouds, newest);
  }

  bool lookup_transform(const std::string& frame, const ros::Time& stamp, Eigen::Isometry3f& out) {
    try {
      geometry_msgs::TransformStamped tf_msg;
      if (frame == output_frame_) {
        out = Eigen::Isometry3f::Identity();
        return true;
      }
      if (tf_buffer_->canTransform(output_frame_, frame, stamp, ros::Duration(tf_timeout_))) {
        tf_msg = tf_buffer_->lookupTransform(output_frame_, frame, stamp, ros::Duration(tf_timeout_));
      } else if (tf_buffer_->canTransform(output_frame_, frame, ros::Time(0), ros::Duration(0.0))) {
        tf_msg = tf_buffer_->lookupTransform(output_frame_, frame, ros::Time(0), ros::Duration(0.0));
      } else {
        ROS_WARN_THROTTLE(
          5.0, "[farlio_multi_lidar] No transform %s -> %s", frame.c_str(), output_frame_.c_str());
        return false;
      }
      const auto& t = tf_msg.transform;
      Eigen::Quaternionf q(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z);
      q.normalize();
      out = Eigen::Isometry3f::Identity();
      out.linear() = q.toRotationMatrix();
      out.translation() = Eigen::Vector3f(t.translation.x, t.translation.y, t.translation.z);
      return true;
    } catch (const std::exception& e) {
      ROS_WARN_THROTTLE(5.0, "[farlio_multi_lidar] tf lookup failed: %s", e.what());
      return false;
    }
  }

  // Find the per-point time field, mirroring FAR-LIO's conventions.
  static std::optional<std::variant<sensor_msgs::PointCloud2ConstIterator<uint32_t>,
                                    sensor_msgs::PointCloud2ConstIterator<float>,
                                    sensor_msgs::PointCloud2ConstIterator<double>>>
  time_iterator(const sensor_msgs::PointCloud2& msg) {
    for (const auto& f : msg.fields) {
      if (f.name != "t" && f.name != "timestamp" && f.name != "time" && f.name != "offset_time") {
        continue;
      }
      if (f.datatype == sensor_msgs::PointField::UINT32) {
        return sensor_msgs::PointCloud2ConstIterator<uint32_t>(msg, f.name);
      }
      if (f.datatype == sensor_msgs::PointField::FLOAT32) {
        return sensor_msgs::PointCloud2ConstIterator<float>(msg, f.name);
      }
      if (f.datatype == sensor_msgs::PointField::FLOAT64) {
        return sensor_msgs::PointCloud2ConstIterator<double>(msg, f.name);
      }
    }
    return std::nullopt;
  }

  void merge_and_publish(const std::vector<Buffered>& clouds, std::uint64_t newest_ns) {
    const bool have_intensity = [&]() {
      for (const auto& c : clouds) {
        bool found = false;
        for (const auto& f : c.msg->fields) {
          if (f.name == "intensity" || f.name == "i") found = true;
        }
        if (!found) return false;
      }
      return true;
    }();
    const bool write_intensity = publish_intensity_ && have_intensity;

    // Compute the total size and cache the transforms.
    std::vector<Eigen::Isometry3f> transforms(clouds.size(), Eigen::Isometry3f::Identity());
    std::size_t total = 0;
    for (std::size_t i = 0; i < clouds.size(); ++i) {
      if (!lookup_transform(clouds[i].msg->header.frame_id, clouds[i].msg->header.stamp, transforms[i])) {
        return;
      }
      total += static_cast<std::size_t>(clouds[i].msg->width) * clouds[i].msg->height;
    }

    sensor_msgs::PointCloud2 out;
    out.header.stamp = ros::Time().fromNSec(newest_ns);
    out.header.frame_id = output_frame_;
    out.height = 1;
    out.width = static_cast<uint32_t>(total);
    out.is_bigendian = false;
    out.is_dense = false;
    int offset = 0;
    offset = addPointField(out, "x", 1, sensor_msgs::PointField::FLOAT32, offset);
    offset = addPointField(out, "y", 1, sensor_msgs::PointField::FLOAT32, offset);
    offset = addPointField(out, "z", 1, sensor_msgs::PointField::FLOAT32, offset);
    if (write_intensity) {
      offset = addPointField(out, "intensity", 1, sensor_msgs::PointField::FLOAT32, offset);
    }
    // FAR-LIO reads a FLOAT32 "time" as seconds relative to the header stamp.
    offset = addPointField(out, "time", 1, sensor_msgs::PointField::FLOAT32, offset);
    out.point_step = static_cast<uint32_t>(offset);
    out.row_step = out.point_step * out.width;
    out.data.assign(static_cast<std::size_t>(out.row_step), 0);

    sensor_msgs::PointCloud2Iterator<float> ox(out, "x");
    sensor_msgs::PointCloud2Iterator<float> oy(out, "y");
    sensor_msgs::PointCloud2Iterator<float> oz(out, "z");
    std::optional<sensor_msgs::PointCloud2Iterator<float>> oint;
    if (write_intensity) oint.emplace(out, "intensity");
    sensor_msgs::PointCloud2Iterator<float> ot(out, "time");

    const double merged_time = static_cast<double>(newest_ns) * 1e-9;
    for (std::size_t ci = 0; ci < clouds.size(); ++ci) {
      const auto& msg = *clouds[ci].msg;
      const auto& tf = transforms[ci];
      const double cloud_time = msg.header.stamp.toSec();
      const std::size_t n = static_cast<std::size_t>(msg.width) * msg.height;

      sensor_msgs::PointCloud2ConstIterator<float> ix(msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iy(msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iz(msg, "z");
      std::optional<sensor_msgs::PointCloud2ConstIterator<float>> iint;
      const bool in_has_intensity = [&]() {
        for (const auto& f : msg.fields) {
          if (f.name == "intensity" || f.name == "i") return f.datatype == sensor_msgs::PointField::FLOAT32;
        }
        return false;
      }();
      if (write_intensity && in_has_intensity) {
        iint.emplace(msg, [&]() {
          for (const auto& f : msg.fields) {
            if (f.name == "intensity" || f.name == "i") return f.name;
          }
          return std::string("intensity");
        }());
      }
      auto itime = time_iterator(msg);

      for (std::size_t k = 0; k < n; ++k, ++ix, ++iy, ++iz) {
        const Eigen::Vector3f p = tf * Eigen::Vector3f(*ix, *iy, *iz);
        *ox = p.x();
        *oy = p.y();
        *oz = p.z();
        ++ox;
        ++oy;
        ++oz;
        if (write_intensity) {
          **oint = (iint && in_has_intensity) ? **iint : 0.0f;
          ++(*oint);
          if (iint) ++(*iint);
        }
        double point_time = cloud_time;
        if (itime) {
          std::visit(
            [&](auto& it) {
              using raw_t = std::decay_t<decltype(*it)>;
              if constexpr (std::is_same_v<raw_t, uint32_t>) {
                point_time = cloud_time + static_cast<double>(*it) * 1e-9;
              } else if constexpr (std::is_same_v<raw_t, float>) {
                point_time = cloud_time + static_cast<double>(*it);
              } else {
                point_time = static_cast<double>(*it);
              }
              ++it;
            },
            *itime);
        }
        *ot = static_cast<float>(point_time - merged_time);
        ++ot;
      }
    }
    pub_.publish(out);
    ROS_INFO_THROTTLE(
      5.0, "[farlio_multi_lidar] merged %zu clouds, %zu points, span %.1f ms", clouds.size(), total,
      1000.0 * static_cast<double>(newest_ns - clouds.front().stamp_ns) * 1e-9);
  }

  ros::NodeHandle nh_, pnh_;
  std::string output_topic_{"/merged_points"};
  std::string output_frame_{"base_link"};
  double max_stamp_delta_{0.02};
  double tf_timeout_{0.05};
  bool publish_intensity_{true};
  int queue_size_{5};

  std::vector<std::string> topics_;
  std::vector<ros::Subscriber> subs_;
  std::map<std::string, Buffered> buffer_;
  std::mutex mutex_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  ros::Publisher pub_;
};

}  // namespace farlio_ros1

int main(int argc, char** argv) {
  ros::init(argc, argv, "farlio_multi_lidar");
  farlio_ros1::MultiLidarMerger merger;
  ros::spin();
  return 0;
}
