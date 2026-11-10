// FAR-LIO ROS 1 node (port of ros_packages/tam_odometry, ROS 2 Jazzy -> ROS 1 Noetic)
//
// The registration pipeline (lib_cpp/odometry_cpp) is used unmodified; this node
// only replaces the ROS 2 plumbing (rclcpp components, param_management_ros2,
// watchdog, TSL publisher, tum_map_msgs service) with roscpp.
//
// Licensed under the Apache License, Version 2.0.
#include <ros/ros.h>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <farlio_msgs/GetPointCloudMap.h>
#include <std_msgs/Header.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "farlio_odometry/config_types.hpp"
#include "farlio_odometry/conversions.hpp"
#include "farlio_odometry/estimation/topic_watchdog.hpp"
#include "farlio_odometry/estimation/tsl_publisher.hpp"
#include "odometry_pipeline/odometry_pipeline.hpp"
#include "param_management_cpp/initialization.hpp"
#include "param_management_cpp/param_reference_manager.hpp"
#include "tsl_logger_cpp/reference_logger.hpp"

namespace farlio_ros1 {
namespace types = tam::core::state::types;

/**
 * @brief Hierarchical YAML -> flat "a.b.c" parameter map for tam::pmg.
 *
 * Scalar types are mapped onto param_value_variant_t. Sequences become
 * std::vector<double> / std::vector<std::int64_t> / std::vector<std::string> /
 * std::vector<bool>; a sequence of mixed / empty content defaults to double.
 */
inline void flatten_yaml(
  const YAML::Node& node, const std::string& prefix,
  std::vector<std::pair<std::string, tam::pmg::param_value_variant_t>>& out)
{
  if (node.IsMap()) {
    for (const auto& kv : node) {
      const std::string key = kv.first.as<std::string>();
      const std::string full = prefix.empty() ? key : prefix + "." + key;
      flatten_yaml(kv.second, full, out);
    }
    return;
  }
  if (node.IsSequence()) {
    if (prefix.empty()) return;
    bool all_bool = true, all_int = true, all_num = true, all_str = true;
    for (const auto& e : node) {
      if (!e.IsScalar()) {
        all_bool = all_int = all_num = all_str = false;
        continue;
      }
      const std::string v = e.Scalar();
      if (v != "true" && v != "false" && v != "True" && v != "False") all_bool = false;
      try {
        std::size_t pos = 0;
        std::stoll(v, &pos);
        if (pos != v.size()) all_int = false;
      } catch (...) {
        all_int = false;
      }
      try {
        std::size_t pos = 0;
        std::stod(v, &pos);
        if (pos != v.size()) all_num = false;
      } catch (...) {
        all_num = false;
      }
      try {
        (void)std::stod(v);
      } catch (...) {
        all_str = false;
      }
    }
    if (all_bool) {
      std::vector<bool> v;
      for (const auto& e : node) v.push_back(e.as<bool>());
      out.emplace_back(prefix, v);
    } else if (all_int) {
      std::vector<std::int64_t> v;
      for (const auto& e : node) v.push_back(e.as<std::int64_t>());
      out.emplace_back(prefix, v);
    } else if (all_num) {
      std::vector<double> v;
      for (const auto& e : node) v.push_back(e.as<double>());
      out.emplace_back(prefix, v);
    } else {
      std::vector<std::string> v;
      for (const auto& e : node) v.push_back(e.as<std::string>());
      out.emplace_back(prefix, v);
    }
    return;
  }
  if (!node.IsScalar() || prefix.empty()) return;

  const std::string raw = node.Scalar();
  // YAML::Node::Tag() is not reliable across yaml-cpp versions; infer by content.
  if (raw == "true" || raw == "True" || raw == "false" || raw == "False") {
    out.emplace_back(prefix, node.as<bool>());
    return;
  }
  try {
    std::size_t pos = 0;
    const std::int64_t v = std::stoll(raw, &pos);
    if (pos == raw.size()) {
      out.emplace_back(prefix, v);
      return;
    }
  } catch (...) {
  }
  try {
    std::size_t pos = 0;
    const double v = std::stod(raw, &pos);
    if (pos == raw.size()) {
      out.emplace_back(prefix, v);
      return;
    }
  } catch (...) {
  }
  out.emplace_back(prefix, raw);
}

inline Eigen::Isometry3f transform2eigen(const geometry_msgs::TransformStamped& tf)
{
  const auto& t = tf.transform;
  Eigen::Quaternionf q(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z);
  q.normalize();
  Eigen::Isometry3f iso = Eigen::Isometry3f::Identity();
  iso.linear() = q.toRotationMatrix();
  iso.translation() = Eigen::Vector3f(t.translation.x, t.translation.y, t.translation.z);
  return iso;
}

inline std::uint64_t stamp2ns(const ros::Time& stamp)
{
  return static_cast<std::uint64_t>(stamp.sec) * 1000000000ULL +
         static_cast<std::uint64_t>(stamp.nsec);
}

template <typename TConfig>
class OdometryNode {
 public:
  OdometryNode(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& config_file)
      : nh_(nh), pnh_(pnh) {
    load_config(config_file);
    read_node_config();

    // Register all pipeline parameters (overrides come from tam::pmg::init()).
    pmg_ = std::make_shared<tam::pmg::ParamReferenceManager>();
    logger_ = std::make_shared<tam::tsl::ReferenceLogger>();
    pipeline_ = tam::core::state::OdometryPipeline<TConfig>::from_config(pmg_.get(), logger_.get());
    pipeline_->init();

    odom_pub_ = nh_.advertise<nav_msgs::Odometry>(output_odom_, 10);
    path_pub_ = nh_.advertise<nav_msgs::Path>(output_odom_ + "_path", 1);
    debug_frame_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("frame_registration", 1);
    debug_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("map", 1, true);
    status_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>(output_odom_ + "_status", 1);

    save_srv_ = nh_.advertiseService<std_srvs::Trigger::Request, std_srvs::Trigger::Response>(
      save_map_service_, [this](std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
        return save_map(res);
      });

    tsl_pub_ = std::make_unique<TslPublisher>(pnh_, logger_);
    watchdog_.add(input_pointcloud_, 0.5, [this](bool timeout, double) {
      if (timeout) publish_timeout_status();
    });
    if (pipeline_->get_diagnostic_config().check_input_status && !input_pointcloud_status_.empty()) {
      input_status_sub_ = nh_.subscribe<diagnostic_msgs::DiagnosticArray>(
        input_pointcloud_status_, 10,
        [this](const diagnostic_msgs::DiagnosticArray::ConstPtr& msg) {
          types::DiagnosticStatus status{};
          if (!msg->status.empty()) {
            status.level = static_cast<types::DiagnosticLevel>(msg->status[0].level);
            status.message = msg->status[0].message;
          }
          pipeline_->set_input_status(status);
        });
    }
    if (!update_map_srv_.empty()) {
      update_map_client_ = nh_.serviceClient<farlio_msgs::GetPointCloudMap>(update_map_srv_);
      update_ready_sub_ = nh_.subscribe<std_msgs::Header>(
        update_map_srv_ + "_ready", 10, &OdometryNode::update_ready_callback, this);
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>();
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>();

    if (!pipeline_->get_config().update_map) {
      load_prior_map();
    } else {
      pipeline_->init_model_config();
      ready_ = true;
    }
    path_.header.frame_id = odom_frame_;

    // Static sensor extrinsics child_frame <- cloud_frame
    if (!wait_for_cloud_transform()) {
      ROS_ERROR("[farlio_odometry] Static transform %s -> %s not available; will retry on first scan.",
                cloud_frame_.c_str(), child_frame_.c_str());
    } else {
      ROS_INFO("[farlio_odometry] Static transform %s -> %s acquired.", cloud_frame_.c_str(),
               child_frame_.c_str());
    }

    sub_ = nh_.subscribe(input_pointcloud_, 10, &OdometryNode::cloud_callback, this);
    ROS_INFO("[farlio_odometry] Listening on %s (cloud frame %s, odom frame %s), publishing %s",
             input_pointcloud_.c_str(), cloud_frame_.c_str(), odom_frame_.c_str(),
             output_odom_.c_str());
  }

 private:
  // ---------------------------------------------------------------------------
  // Configuration
  // ---------------------------------------------------------------------------
  void load_config(const std::string& config_file) {
    YAML::Node root = YAML::LoadFile(config_file);
    std::vector<std::pair<std::string, tam::pmg::param_value_variant_t>> params;
    flatten_yaml(root, "", params);
    ROS_INFO("[farlio_odometry] Loaded %zu parameters from %s", params.size(), config_file.c_str());
    tam::pmg::init(params);

    // Node-level configuration is read from the same file.
    YAML::Node n = root["node"];
    auto get_s = [&](const char* key, const std::string& def) {
      return n[key] ? n[key].as<std::string>() : def;
    };
    odom_frame_ = get_s("odom_frame", "local_cartesian");
    child_frame_ = get_s("child_frame", "base_link");
    cloud_frame_ = get_s("cloud_frame", "lidar_link");
    input_pointcloud_ = get_s("input_pointcloud", "/points_raw");
    output_odom_ = get_s("output_odom", "lidar_odometry");
    input_map_ = get_s("input_map", "");
    update_map_srv_ = get_s("update_map_srv", "");
    input_pointcloud_status_ = get_s("input_pointcloud_status", "");
    wait_tf_ = n["wait_tf"] ? n["wait_tf"].as<bool>() : false;
    max_cloud_range_ = n["max_cloud_range"] ? n["max_cloud_range"].as<double>() : 0.0;

    YAML::Node loc = root["localization"];
    if (loc && loc["initial_pose"] && loc["initial_pose"].size() == 6) {
      has_initial_pose_ = true;
      for (std::size_t i = 0; i < 6; ++i) initial_pose_[i] = loc["initial_pose"][i].as<double>();
    }
    YAML::Node out = root["output"];
    save_map_path_ = out && out["map_path"] ? out["map_path"].as<std::string>() : "";
  }

  void read_node_config() {
    pnh_.param<std::string>("pipeline_type", pipeline_type_, "gicp_cv");
    pnh_.param<std::string>("save_map_service", save_map_service_, "save_map");
    pnh_.param<bool>("publish_debug_clouds", publish_debug_clouds_, true);
    if (save_map_path_.empty()) {
      pnh_.param<std::string>("map_save_path", save_map_path_, "/tmp/farlio_map.pcd");
    }
  }

  // ---------------------------------------------------------------------------
  // Map / localization
  // ---------------------------------------------------------------------------
  void load_prior_map() {
    std::string path = input_map_;
    if (path.empty()) {
      ROS_ERROR("[farlio_odometry] update_map=false but node.input_map is empty.");
      return;
    }
    if (path.back() == '/') path += "map.pcd";
    if (path.size() < 4 || path.substr(path.size() - 4) != ".pcd") {
      // Upstream: input_map may name a GetPointCloudMap service instead of a file.
      map_client_ = nh_.serviceClient<farlio_msgs::GetPointCloudMap>(input_map_);
      map_timer_ = nh_.createTimer(
        ros::Duration(5.0), &OdometryNode::init_map_callback, this);
      ROS_INFO(
        "[farlio_odometry] Localization mode: requesting the prior map from service '%s'",
        input_map_.c_str());
      return;
    }
    const auto points = conversions::load_static_map<types::Point_XYZ>(path);
    if (points.empty()) {
      ROS_ERROR("[farlio_odometry] Prior map '%s' is empty.", path.c_str());
      return;
    }
    apply_prior_map(points);
    finish_localization_init();
  }

  void apply_prior_map(const std::vector<types::Point<types::Point_XYZ>>& points) {
    pipeline_->add_points(points);
    ROS_INFO("[farlio_odometry] Prior map loaded: %zu points", points.size());
  }

  void finish_localization_init() {
    // Initialize the motion model before overriding its pose with the prior.
    pipeline_->init_model_config();

    if (has_initial_pose_) {
      types::PoseStamped pose;
      pose.stamp = stamp2ns(ros::Time::now());
      const double x = initial_pose_[0], y = initial_pose_[1], z = initial_pose_[2];
      const double roll = initial_pose_[3], pitch = initial_pose_[4], yaw = initial_pose_[5];
      const Eigen::AngleAxisf rx(static_cast<float>(roll), Eigen::Vector3f::UnitX());
      const Eigen::AngleAxisf ry(static_cast<float>(pitch), Eigen::Vector3f::UnitY());
      const Eigen::AngleAxisf rz(static_cast<float>(yaw), Eigen::Vector3f::UnitZ());
      const Eigen::Quaternionf q(rz * ry * rx);
      pose.pose = Sophus::SE3f(
        Sophus::SE3f::QuaternionType(q.w(), q.x(), q.y(), q.z()),
        Sophus::SE3f::Point(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)));
      pipeline_->set_pose_model(pose, true);
      ROS_INFO("[farlio_odometry] Initial pose set from config: [%.2f %.2f %.2f %.2f %.2f %.2f]", x,
               y, z, roll, pitch, yaw);
      ready_ = true;
    } else if (wait_tf_) {
      ROS_INFO("[farlio_odometry] Waiting for initial pose from TF %s -> %s ...",
               odom_frame_.c_str(), child_frame_.c_str());
    } else {
      ROS_ERROR(
        "[farlio_odometry] Localization mode needs localization.initial_pose or node.wait_tf=true.");
    }
  }

  void init_map_callback(const ros::TimerEvent&) {
    if (ready_ && map_loaded_) {
      if (map_timer_.isValid()) map_timer_.stop();
      return;
    }
    if (!map_client_.exists()) {
      ROS_WARN_THROTTLE(5.0, "[farlio_odometry] Map service '%s' not available.",
                        map_client_.getService().c_str());
      return;
    }
    farlio_msgs::GetPointCloudMap srv;
    srv.request.header.stamp = ros::Time::now();
    srv.request.header.frame_id = odom_frame_;
    srv.request.voxel_size = static_cast<float>(pipeline_->get_map_resolution());
    if (!map_client_.call(srv) || !srv.response.success) {
      ROS_WARN_THROTTLE(5.0, "[farlio_odometry] Map service call failed.");
      return;
    }
    auto points = conversions::cloud2eigen<types::Point_XYZ>(
      srv.response.map, Eigen::Isometry3f::Identity());
    if (points.empty()) {
      ROS_WARN_THROTTLE(5.0, "[farlio_odometry] Map service returned an empty map.");
      return;
    }
    apply_prior_map(points);
    map_loaded_ = true;
    if (map_timer_.isValid()) map_timer_.stop();
    finish_localization_init();
  }

  void update_ready_callback(const std_msgs::Header::ConstPtr&) {
    if (!ready_) return;
    bool expected = false;
    if (!update_in_progress_.compare_exchange_strong(expected, true)) {
      ROS_WARN("[farlio_odometry] Map update already in progress - skipping");
      return;
    }
    std::thread([this]() {
      farlio_msgs::GetPointCloudMap srv;
      srv.request.header.stamp = ros::Time::now();
      srv.request.header.frame_id = odom_frame_;
      srv.request.voxel_size = static_cast<float>(pipeline_->get_map_resolution());
      if (update_map_client_.call(srv) && srv.response.success) {
        auto points = conversions::cloud2eigen<types::Point_XYZ>(
          srv.response.map, Eigen::Isometry3f::Identity());
        {
          std::lock_guard<std::mutex> lock(pipeline_->get_map_mutex());
          pipeline_->add_points(
            points, TConfig::MAX_POINTS_PER_VOXEL, 1, TConfig::NUM_NEIGHBORS, false);
          pipeline_->request_map_switch();
        }
        ROS_INFO("[farlio_odometry] Asynchronous map update applied (%zu points)", points.size());
      } else {
        ROS_WARN("[farlio_odometry] Asynchronous map update failed");
      }
      update_in_progress_.store(false);
    }).detach();
  }

  void publish_timeout_status() {
    diagnostic_msgs::DiagnosticArray arr;
    arr.header.stamp = ros::Time::now();
    diagnostic_msgs::DiagnosticStatus st;
    st.name = "farlio_odometry";
    st.hardware_id = "far-lio";
    st.level = diagnostic_msgs::DiagnosticStatus::ERROR;
    st.message = "pointcloud timeout";
    arr.status.push_back(st);
    status_pub_.publish(arr);
  }

  bool save_map(std_srvs::Trigger::Response& res) {
    const auto points = pipeline_->get_map();
    if (points.empty()) {
      res.success = false;
      res.message = "Map is empty";
      return true;
    }
    pcl::PointCloud<pcl::PointXYZI> cloud;
    cloud.reserve(points.size());
    for (const auto& p : points) {
      pcl::PointXYZI pt;
      pt.x = p.pos.x();
      pt.y = p.pos.y();
      pt.z = p.pos.z();
      pt.intensity = 0.0f;
      cloud.push_back(pt);
    }
    const std::string path = save_map_path_.empty() ? "/tmp/farlio_map.pcd" : save_map_path_;
    if (pcl::io::savePCDFileBinary(path, cloud) != 0) {
      res.success = false;
      res.message = "Failed to write " + path;
      return true;
    }
    res.success = true;
    res.message = "Saved " + std::to_string(cloud.size()) + " points to " + path;
    ROS_INFO("[farlio_odometry] %s", res.message.c_str());
    return true;
  }

  // ---------------------------------------------------------------------------
  // TF
  // ---------------------------------------------------------------------------
  bool wait_for_cloud_transform() {
    const ros::Time start = ros::Time::now();
    while (ros::ok()) {
      if (tf_buffer_->canTransform(
            child_frame_, cloud_frame_, ros::Time(0), ros::Duration(0.05))) {
        pc_transform_ = transform2eigen(
          tf_buffer_->lookupTransform(child_frame_, cloud_frame_, ros::Time(0), ros::Duration(0.05)));
        has_cloud_transform_ = true;
        return true;
      }
      if ((ros::Time::now() - start).toSec() > 5.0) return false;
      ros::Duration(0.1).sleep();
    }
    return false;
  }

  void set_undistortion_poses(std::uint64_t stamp) {
    if (!pipeline_->get_config().undistort) return;
    constexpr std::uint64_t INTERVAL_NS = 10'000'000ULL;  // 10 ms
    const std::uint64_t latest = stamp - stamp % INTERVAL_NS;
    std::uint64_t newest_set = distortion_pose_stamp_;
    bool gap = false;
    for (int i = 19; i >= 0; --i) {
      const std::uint64_t sample_ns = latest - static_cast<std::uint64_t>(i) * INTERVAL_NS;
      if (sample_ns <= distortion_pose_stamp_) continue;
      const ros::Time sample_time;
      ros::Time t;
      t.fromNSec(sample_ns);
      if (tf_buffer_->canTransform(odom_frame_, child_frame_, t, ros::Duration(0))) {
        pipeline_->set_pose_undistortion(
          conversions::transform2pose(tf_buffer_->lookupTransform(odom_frame_, child_frame_, t)));
        if (!gap) newest_set = sample_ns;
      } else {
        gap = true;
      }
    }
    distortion_pose_stamp_ = newest_set;
  }

  bool lookup_initial_guess(const ros::Time& stamp, types::PoseStamped& guess) {
    if (tf_buffer_->canTransform(odom_frame_, child_frame_, stamp, ros::Duration(0))) {
      guess = conversions::transform2pose(
        tf_buffer_->lookupTransform(odom_frame_, child_frame_, stamp));
      return true;
    }
    if (tf_buffer_->canTransform(odom_frame_, child_frame_, ros::Time(0), ros::Duration(0))) {
      guess = conversions::transform2pose(
        tf_buffer_->lookupTransform(odom_frame_, child_frame_, ros::Time(0)));
      return true;
    }
    return false;
  }

  // ---------------------------------------------------------------------------
  // Main callback
  // ---------------------------------------------------------------------------
  void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    watchdog_.update(input_pointcloud_);
    if (!has_cloud_transform_ && !wait_for_cloud_transform()) return;

    if (msg->header.frame_id != cloud_frame_) {
      ROS_WARN_THROTTLE(
        5.0, "[farlio_odometry] Cloud frame '%s' differs from configured cloud_frame '%s'",
        msg->header.frame_id.c_str(), cloud_frame_.c_str());
    }
    if (!ready_ && wait_tf_) {
      types::PoseStamped guess;
      if (!lookup_initial_guess(msg->header.stamp, guess)) {
        ROS_WARN_THROTTLE(2.0, "[farlio_odometry] Waiting for initial pose from TF ...");
        return;
      }
      pipeline_->set_pose_model(guess, true);
      ready_ = true;
      ROS_INFO("[farlio_odometry] Initial pose received from TF, starting registration.");
    }
    if (!ready_) return;

    const auto t_start = std::chrono::high_resolution_clock::now();
    auto frame = conversions::cloud2eigen<TConfig>(*msg, pc_transform_);

    if (max_cloud_range_ > 0.0) {
      const float r2 = static_cast<float>(max_cloud_range_ * max_cloud_range_);
      frame.erase(
        std::remove_if(
          frame.begin(), frame.end(),
          [r2](const auto& p) { return p.pos.squaredNorm() > r2 || !p.pos.allFinite(); }),
        frame.end());
    }

    if constexpr (TConfig::MODEL == types::ModelType::EXTERNALGUESS) {
      types::PoseStamped guess;
      const bool valid = lookup_initial_guess(msg->header.stamp, guess);
      if (!valid) {
        const auto odom = pipeline_->get_odometry();
        guess.pose = odom.pose.pose;
        guess.stamp = stamp2ns(msg->header.stamp);
      }
      pipeline_->set_pose_model(guess, valid);
    }

    set_undistortion_poses(stamp2ns(msg->header.stamp));
    types::Odometry odom = pipeline_->register_frame(frame, stamp2ns(msg->header.stamp));

    const double ms =
      std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start)
        .count();

    const nav_msgs::Odometry odom_msg =
      conversions::odom2msg(odom, odom_frame_, child_frame_);
    odom_pub_.publish(odom_msg);

    // Only the node that owns the state estimate may broadcast the dynamic
    // transform. In external-guess mode the EKF publishes it (upstream does the
    // same), otherwise the two TF sources would fight and the initial guess would
    // become stale ("large time diff").
    if constexpr (TConfig::MODEL != types::ModelType::EXTERNALGUESS) {
      geometry_msgs::TransformStamped tf_msg;
      tf_msg.header.stamp = msg->header.stamp;
      tf_msg.header.frame_id = odom_frame_;
      tf_msg.child_frame_id = child_frame_;
      tf_msg.transform = conversions::sophus2transform(odom.pose.pose);
      tf_broadcaster_->sendTransform(tf_msg);
    }

    // Path for RViz / evaluation
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = odom_msg.header;
    pose_stamped.pose = odom_msg.pose.pose;
    path_.header.stamp = odom_msg.header.stamp;
    path_.poses.push_back(pose_stamped);
    path_pub_.publish(path_);

    publish_status(odom, ms);

    const bool debug = pipeline_->get_config().debug_mode && publish_debug_clouds_;
    if (debug) {
      std_msgs::Header header;
      header.stamp = msg->header.stamp;
      header.frame_id = child_frame_;
      debug_frame_pub_.publish(conversions::eigen2cloud<TConfig>(pipeline_->get_frame(), header));
      if (pipeline_->get_config().update_map) {
        header.frame_id = odom_frame_;
        debug_map_pub_.publish(conversions::eigen2cloud<TConfig>(pipeline_->get_map(), header));
      }
    }

    const double now_sec = ros::Time::now().toSec();
    if (now_sec - last_tsl_sec_ >= 0.1) {
      tsl_pub_->trigger();
      last_tsl_sec_ = now_sec;
    }

    const auto reg = pipeline_->get_registration_status();
    ROS_INFO_THROTTLE(
      2.0,
      "[farlio_odometry] %zu pts | %zu it | %.1f ms | converged=%d | pos=(%.2f, %.2f, %.2f)",
      frame.size(), reg.num_iter, ms, static_cast<int>(reg.converged), odom.pose.pose.translation().x(),
      odom.pose.pose.translation().y(), odom.pose.pose.translation().z());
  }

  void publish_status(const types::Odometry& odom, double ms) {
    diagnostic_msgs::DiagnosticArray arr;
    arr.header.stamp = ros::Time().fromNSec(odom.stamp);
    diagnostic_msgs::DiagnosticStatus st;
    st.name = "farlio_odometry";
    st.hardware_id = "far-lio";
    st.level = static_cast<std::int8_t>(odom.status.level);
    st.message = odom.status.message;
    st.values.emplace_back();
    st.values.back().key = "processing_time_ms";
    st.values.back().value = std::to_string(ms);
    for (const auto& kv : odom.status.key_values) {
      diagnostic_msgs::KeyValue v;
      v.key = kv.first;
      v.value = kv.second;
      st.values.push_back(v);
    }
    arr.status.push_back(st);
    status_pub_.publish(arr);
  }

  ros::NodeHandle nh_, pnh_;
  std::shared_ptr<tam::pmg::ParamReferenceManager> pmg_;
  std::shared_ptr<tam::tsl::ReferenceLogger> logger_;
  std::unique_ptr<tam::core::state::OdometryPipeline<TConfig>> pipeline_;

  std::string pipeline_type_{"gicp_cv"};
  std::string odom_frame_{"local_cartesian"};
  std::string child_frame_{"base_link"};
  std::string cloud_frame_{"lidar_link"};
  std::string input_pointcloud_{"/points_raw"};
  std::string output_odom_{"lidar_odometry"};
  std::string input_map_{};
  std::string save_map_path_{};
  std::string update_map_srv_{};
  std::string input_pointcloud_status_{};
  std::string save_map_service_{"save_map"};
  bool wait_tf_{false};
  bool has_initial_pose_{false};
  bool publish_debug_clouds_{true};
  bool ready_{false};
  bool has_cloud_transform_{false};
  double max_cloud_range_{0.0};
  double initial_pose_[6] = {0, 0, 0, 0, 0, 0};

  Eigen::Isometry3f pc_transform_{Eigen::Isometry3f::Identity()};
  std::uint64_t distortion_pose_stamp_{0};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  ros::Publisher odom_pub_, path_pub_, debug_frame_pub_, debug_map_pub_, status_pub_;
  ros::Subscriber sub_, input_status_sub_, update_ready_sub_;
  ros::ServiceServer save_srv_;
  ros::ServiceClient map_client_, update_map_client_;
  ros::Timer map_timer_;
  std::atomic<bool> update_in_progress_{false};
  bool map_loaded_{false};
  double last_tsl_sec_{0.0};
  std::unique_ptr<TslPublisher> tsl_pub_;
  tam::core::TopicWatchdog watchdog_;
  nav_msgs::Path path_;
};

template <typename TConfig>
int run_node(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& yaml_file) {
  OdometryNode<TConfig> node(nh, pnh, yaml_file);
  ros::spin();
  return 0;
}
}  // namespace farlio_ros1

int main(int argc, char** argv) {
  ros::init(argc, argv, "farlio_odometry");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string yaml_file;
  if (!pnh.getParam("config_file", yaml_file)) {
    ROS_FATAL("[farlio_odometry] ~config_file is required.");
    return 1;
  }
  std::string type;
  pnh.param<std::string>("pipeline_type", type, "gicp_cv");

  try {
    if (type == "gicp_cv") {
      return farlio_ros1::run_node<farlio_ros1::GICP_CV>(nh, pnh, yaml_file);
    } else if (type == "icp_cv") {
      return farlio_ros1::run_node<tam::core::state::types::ICP_CV>(nh, pnh, yaml_file);
    } else if (type == "gicp_ext") {
      return farlio_ros1::run_node<tam::core::state::types::GICP_EXT>(nh, pnh, yaml_file);
    } else if (type == "icp_ext") {
      return farlio_ros1::run_node<tam::core::state::types::ICP_EXT>(nh, pnh, yaml_file);
    }
    ROS_FATAL("[farlio_odometry] Unknown pipeline_type '%s'.", type.c_str());
    return 1;
  } catch (const std::exception& e) {
    ROS_FATAL("[farlio_odometry] %s", e.what());
    return 1;
  }
}
