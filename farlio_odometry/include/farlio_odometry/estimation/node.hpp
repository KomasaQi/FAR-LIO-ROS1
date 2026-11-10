// Licensed under the Apache License, Version 2.0.
//
// ROS 1 port of tam_state_estimation_node/node.hpp.
//
// The estimation logic lives in the unmodified TUM library
// (tam_state_estimation / state_estimation_base). This class only provides the
// ROS 1 plumbing: declarative sensor configuration, subscriptions, delay
// compensation, watchdog, diagnostics, TF and publishing.
#pragma once

#include <diagnostic_msgs/DiagnosticArray.h>
#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

#include <farlio_msgs/SteeringReport.h>
#include <farlio_msgs/TUMFloat64PerWheelStamped.h>
#include <farlio_msgs/TSLValues.h>

#include "farlio_odometry/estimation/helper.hpp"
#include "farlio_odometry/estimation/node_monitor.hpp"
#include "farlio_odometry/estimation/ros1_conversions.hpp"
#include "farlio_odometry/estimation/topic_watchdog.hpp"
#include "farlio_odometry/estimation/tsl_publisher.hpp"
#include "farlio_odometry/estimation/types.hpp"
#include "farlio_odometry/yaml_params.hpp"

#include "param_management_cpp/param_manager_composer.hpp"
#include "param_management_cpp/param_value_manager.hpp"
#include "state_estimation_base/state_estimation_base.hpp"
#include "state_estimation_constants/EKF_2D.hpp"
#include "state_estimation_constants/EKF_3D.hpp"
#include "tam_state_estimation/state_estimation.hpp"
#include "tum_helpers_cpp/delay_compensation.hpp"
#include "tum_helpers_cpp/rotations.hpp"
#include "tum_type_conversions_ros_cpp/tum_type_conversions.hpp"
#include "tum_types_cpp/common.hpp"
#include "tum_types_cpp/control.hpp"

namespace farlio_ros1 {

template <class TConfig, tam::core::state::VehicleModel TModel>
class StateEstimationNode {
 public:
  StateEstimationNode(
    ros::NodeHandle& nh, ros::NodeHandle& pnh,
    std::unique_ptr<tam::core::state::StateEstimationBase>&& state_estimation,
    const YAML::Node& config)
      : nh_(nh), pnh_(pnh), state_estimation_(std::move(state_estimation))
  {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>();
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>();
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Combine node parameters and the estimator's parameter managers.
    param_manager_composer_ = std::make_shared<tam::pmg::ParamManagerComposer>(
      std::vector<tam::pmg::MgmtInterface::SharedPtr>{
        param_manager_, state_estimation_->get_param_handler()});
    tam::pmg::MgmtInterface* param_manager_raw = param_manager_composer_.get();
    declare_and_update_parameters();

    // Parse the declarative measurement / IMU configuration from the YAML file.
    std::vector<std::pair<tam::types::state::measurements::identifier, std::string>>
      identifier_names{};
    std::map<std::string, farlio_ros1::ParamMap> measurement_map, input_map;
    if (config["measurements"] && config["measurements"].IsMap()) {
      for (const auto& kv : config["measurements"]) {
        measurement_map[kv.first.as<std::string>()] = farlio_ros1::parse_param_map(kv.second);
      }
    }
    if (config["inputs"] && config["inputs"]["imus"] && config["inputs"]["imus"].IsMap()) {
      for (const auto& kv : config["inputs"]["imus"]) {
        input_map[kv.first.as<std::string>()] = farlio_ros1::parse_param_map(kv.second);
      }
    }

    tam::core::state::parse_sensor_configs<TConfig>(
      param_manager_raw, measurement_map, input_map, measurement_channels_, input_channels_,
      identifier_names);

    // Assemble measurement noise covariance (R) and outlier bounds.
    std::vector<double> R_int = std::vector<double>(TConfig::MEASUREMENT_VECTOR_SIZE, 1.0e9);
    std::vector<double> outlier_bounds =
      std::vector<double>(TConfig::MEASUREMENT_VECTOR_SIZE, 1.0e9);
    for (const auto& measurement : measurement_channels_) {
      if (measurement.config.position_m.active) {
        const uint8_t begin_offset =
          measurement.config.position_m.num * TConfig::POS_MEASUREMENT_VECTOR_SIZE;
        check_size(begin_offset, measurement.config.position_m.R_init.size(), "position_m",
                   measurement.config.name);
        std::copy(
          measurement.config.position_m.R_init.begin(), measurement.config.position_m.R_init.end(),
          R_int.begin() + begin_offset);
        std::copy(
          measurement.config.position_m.outlier_bounds.begin(),
          measurement.config.position_m.outlier_bounds.end(), outlier_bounds.begin() + begin_offset);
        identifier_names.emplace_back(
          tam::types::state::measurements::identifier{
            tam::types::state::measurements::POS, measurement.config.position_m.num},
          measurement.config.name);
      }
      if (measurement.config.orientation_rad.active) {
        const uint8_t begin_offset = TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION +
                                     measurement.config.orientation_rad.num *
                                       TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE;
        check_size(begin_offset, measurement.config.orientation_rad.R_init.size(),
                   "orientation_rad", measurement.config.name);
        std::copy(
          measurement.config.orientation_rad.R_init.begin(),
          measurement.config.orientation_rad.R_init.end(), R_int.begin() + begin_offset);
        std::copy(
          measurement.config.orientation_rad.outlier_bounds.begin(),
          measurement.config.orientation_rad.outlier_bounds.end(),
          outlier_bounds.begin() + begin_offset);
        identifier_names.emplace_back(
          tam::types::state::measurements::identifier{
            tam::types::state::measurements::ORIENTATION,
            measurement.config.orientation_rad.num},
          measurement.config.name);
      }
      if (measurement.config.linear_velocity_mps.active) {
        const uint8_t begin_offset = TConfig::MEASUREMENT_VECTOR_OFFSET_VEL +
                                     measurement.config.linear_velocity_mps.num *
                                       TConfig::VEL_MEASUREMENT_VECTOR_SIZE;
        check_size(begin_offset, measurement.config.linear_velocity_mps.R_init.size(),
                   "linear_velocity_mps", measurement.config.name);
        std::copy(
          measurement.config.linear_velocity_mps.R_init.begin(),
          measurement.config.linear_velocity_mps.R_init.end(), R_int.begin() + begin_offset);
        std::copy(
          measurement.config.linear_velocity_mps.outlier_bounds.begin(),
          measurement.config.linear_velocity_mps.outlier_bounds.end(),
          outlier_bounds.begin() + begin_offset);
        identifier_names.emplace_back(
          tam::types::state::measurements::identifier{
            tam::types::state::measurements::VEL,
            measurement.config.linear_velocity_mps.num},
          measurement.config.name);
      }
    }

    if (param_manager_raw->get_value("enable.reference_angle_measurement").as_bool()) {
      const uint8_t begin_offset =
        TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION +
        (TConfig::NUM_ORIENTATION_MEASUREMENT - 1) * TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE;
      const auto R_init =
        param_manager_raw->get_value("reference_angles.orientation.R_init").as_double_array();
      const auto bounds = param_manager_raw->get_value("reference_angles.orientation.outlier_bounds")
                            .as_double_array();
      std::copy(R_init.begin(), R_init.end(), R_int.begin() + begin_offset);
      std::copy(bounds.begin(), bounds.end(), outlier_bounds.begin() + begin_offset);
    }

    if (param_manager_raw->get_value("enable.external_orientation_measurement").as_bool()) {
      const uint8_t begin_offset =
        TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION +
        (TConfig::NUM_ORIENTATION_MEASUREMENT - 2) * TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE;
      const auto R_init =
        param_manager_raw->get_value("external_orientation.orientation.R_init").as_double_array();
      const auto bounds =
        param_manager_raw->get_value("external_orientation.orientation.outlier_bounds")
          .as_double_array();
      std::copy(R_init.begin(), R_init.end(), R_int.begin() + begin_offset);
      std::copy(bounds.begin(), bounds.end(), outlier_bounds.begin() + begin_offset);
    }

    if constexpr (TModel != tam::core::state::VehicleModel::Kinematic) {
      const uint8_t begin_offset = TConfig::MEASUREMENT_VECTOR_OFFSET_VEL +
                                   (TConfig::NUM_VEL_MEASUREMENT - 1) *
                                     TConfig::VEL_MEASUREMENT_VECTOR_SIZE;
      const auto R_init =
        param_manager_raw->get_value("vehicle_model.linear_velocity_mps.R_init")
          .as_double_array();
      const auto bounds =
        param_manager_raw->get_value("vehicle_model.linear_velocity_mps.outlier_bounds")
          .as_double_array();
      std::copy(R_init.begin(), R_init.end(), R_int.begin() + begin_offset);
      std::copy(bounds.begin(), bounds.end(), outlier_bounds.begin() + begin_offset);
    }

    param_manager_raw->set_value("kalman_filter.R_init", R_int);
    param_manager_raw->set_value("kalman_filter.outlier_bounds", outlier_bounds);
    state_estimation_->set_input_config(identifier_names);

    // Monitoring.
    monitor_ = std::make_unique<tam::core::NodeMonitor>(nh_, "StateEstimation");
    delay_compensation_ =
      std::make_unique<tam::helpers::DelayCompensation>(1 / TConfig::TS, TConfig::TS);

    // Model update timer (100 Hz).
    timer_ = nh_.createTimer(
      ros::Duration(TConfig::TS), &StateEstimationNode::function_queue_callback, this);

    // Measurement subscriptions.
    for (std::size_t i = 0; i < measurement_channels_.size(); ++i) {
      auto& measurement = measurement_channels_[i];
      if (measurement.config.status_sub.active) {
        status_subs_.push_back(nh_.subscribe<diagnostic_msgs::DiagnosticArray>(
          measurement.config.status_sub.topic, 10,
          [this, i](const diagnostic_msgs::DiagnosticArray::ConstPtr& msg) {
            measurement_status_callback(msg, this->measurement_channels_[i]);
          }));
        watchdog_.add(
          measurement.config.status_sub.topic, measurement.config.status_sub.timeout_ms / 1000.0,
          [this, i](bool timeout, double) {
            if (!timeout) return;
            // A stale status topic invalidates all fusion inputs of this channel.
            const auto& m = measurement_channels_[i];
            if (m.config.position_m.active)
              state_estimation_->set_input_position_status(
                tam::types::ErrorLvl::STALE, m.config.position_m.num);
            if (m.config.orientation_rad.active)
              state_estimation_->set_input_orientation_status(
                tam::types::ErrorLvl::STALE, m.config.orientation_rad.num);
            if (m.config.linear_velocity_mps.active)
              state_estimation_->set_input_linear_velocity_status(
                tam::types::ErrorLvl::STALE, m.config.linear_velocity_mps.num);
          });
      }
      measurement_subs_.push_back(nh_.subscribe<nav_msgs::Odometry>(
        measurement.config.measurement_sub.topic, 10,
        [this, i](const nav_msgs::Odometry::ConstPtr& msg) {
          this->measurement_callback(msg, this->measurement_channels_[i]);
        }));
      watchdog_.add(
        measurement.config.measurement_sub.topic,
        measurement.config.measurement_sub.timeout_ms / 1000.0,
        [this, i](bool timeout, double) {
          this->measurement_timeout_callback(timeout, this->measurement_channels_[i]);
        });
    }

    // IMU subscriptions.
    for (std::size_t i = 0; i < input_channels_.size(); ++i) {
      auto& input = input_channels_[i];
      imu_subs_.push_back(nh_.subscribe<sensor_msgs::Imu>(
        input.config.measurement_sub.topic, 10,
        [this, i](const sensor_msgs::Imu::ConstPtr& msg) {
          this->input_callback(msg, this->input_channels_[i]);
        }));
      watchdog_.add(
        input.config.measurement_sub.topic, input.config.measurement_sub.timeout_ms / 1000.0,
        [this, i](bool timeout, double) {
          this->imu_timeout_callback(timeout, this->input_channels_[i]);
        });
    }

    if constexpr (TModel != tam::core::state::VehicleModel::Kinematic) {
      wheel_sub_ = nh_.subscribe<farlio_msgs::TUMFloat64PerWheelStamped>(
        "/vehicle/sensor/wheelspeed_radps", 10,
        [this](const farlio_msgs::TUMFloat64PerWheelStamped::ConstPtr& msg) {
          const auto input = tam::type_conversions::data_per_wheel_type_from_msg(msg->data);
          state_estimation_->set_input_wheelspeeds(input);
        });
      wheel_status_sub_ = nh_.subscribe<diagnostic_msgs::DiagnosticArray>(
        "/vehicle/sensor/wheelspeed_status", 10,
        [this](const diagnostic_msgs::DiagnosticArray::ConstPtr& msg) {
          if (msg->status.empty()) return;
          state_estimation_->set_input_wheelspeed_status(
            tam::type_conversions::error_type_from_diagnostic_level(msg->status[0].level));
        });
      watchdog_.add("/vehicle/sensor/wheelspeed_radps", 0.5,
                    [this](bool timeout, double) {
                      if (timeout) state_estimation_->set_wheelspeeds_timeout();
                    });
      watchdog_.add("/vehicle/sensor/wheelspeed_status", 0.5,
                    [this](bool timeout, double) {
                      if (timeout) state_estimation_->set_wheelspeeds_timeout();
                    });
    }

    if constexpr (TModel == tam::core::state::VehicleModel::SingleTrack) {
      steering_sub_ = nh_.subscribe<farlio_msgs::SteeringReport>(
        "/vehicle/sensor/steering_report", 10,
        [this](const farlio_msgs::SteeringReport::ConstPtr& msg) {
          state_estimation_->set_input_steering_angle(msg->steering_tire_angle);
          state_estimation_->set_input_steering_angle_status(tam::types::ErrorLvl::OK);
        });
      watchdog_.add("/vehicle/sensor/steering_report", 0.5,
                    [this](bool timeout, double) {
                      if (timeout) state_estimation_->set_steering_angle_timeout();
                    });
    }

    // Output publishers.
    pub_odometry_ = nh_.advertise<nav_msgs::Odometry>("/core/state/odometry", 10);
    pub_acceleration_ =
      nh_.advertise<geometry_msgs::AccelWithCovarianceStamped>("/core/state/acceleration", 10);

    tsl_publisher_ = std::make_unique<TslPublisher>(pnh_, logger_);
    pnh_.param("tsl_publish_rate_hz", tsl_rate_hz_, 10.0);

    ROS_INFO(
      "[StateEstimation] initialized (3D=%d, model=%d, %zu measurement channels, %zu IMUs)",
      static_cast<int>(TConfig::THREE_DIMENSIONAL), static_cast<int>(TModel),
      measurement_channels_.size(), input_channels_.size());
  }

 private:
  static void check_size(uint8_t offset, std::size_t size, const std::string& group,
                         const std::string& name)
  {
    if (offset + size > 1000) {
      throw std::invalid_argument(
        "[StateEstimation]: too many/oversized '" + group + "' fusion configs for " + name);
    }
  }

  void declare_and_update_parameters()
  {
    state_estimation_node_params_.odom_frame_id = param_manager_->declare_and_get_value(
                                                     "node.odom_frame_id", std::string("local_cartesian"),
                                                     tam::pmg::ParameterType::STRING,
                                                     "Frame of the odometry output")
                                                     .as_string();
    state_estimation_node_params_.child_frame_id = param_manager_->declare_and_get_value(
                                                      "node.child_frame_id", std::string("base_link"),
                                                      tam::pmg::ParameterType::STRING,
                                                      "Child frame of the odometry output")
                                                      .as_string();
    state_estimation_node_params_.virtual_covariance_scale =
      param_manager_->declare_and_get_value(
                     "kalman_filter.covariance_adaption.virtual_scale", 10.0,
                     tam::pmg::ParameterType::DOUBLE,
                     "Scaling factor for the covariance of delayed measurements")
        .as_double();
    state_estimation_node_params_.average_input_delay_ms = param_manager_->declare_and_get_value(
                                                              "inputs.average_delay_ms",
                                                              std::int64_t{0},
                                                              tam::pmg::ParameterType::INTEGER,
                                                              "Average expected IMU delay [ms]")
                                                              .as_int();
    previous_param_state_hash_ = param_manager_->get_state_hash();
  }

  // -------------------------------------------------------------------------
  // Timer
  // -------------------------------------------------------------------------
  void function_queue_callback(const ros::TimerEvent&)
  {
    if (param_manager_->get_state_hash() != previous_param_state_hash_) {
      declare_and_update_parameters();
    }
    watchdog_.check_timeouts();
    model_update_callback();

    std_msgs::Header header;
    header.stamp = time_pub_.isZero() ? ros::Time::now() : time_pub_;
    header.frame_id = state_estimation_node_params_.odom_frame_id;
    monitor_->update(header);
  }

  void model_update_callback()
  {
    const auto start_delay_comp = std::chrono::high_resolution_clock::now();
    const double covariance_scale = state_estimation_node_params_.virtual_covariance_scale;
    for (auto& measurement : measurement_channels_) {
      if (!measurement.received) continue;
      tam::types::control::Odometry input = measurement.odometry;
      const double time_diff = (ros::Time::now() - measurement.measurement_stamp).toSec();
      if (delay_compensation_->compensate(input, time_diff)) {
        std::transform(
          measurement.odometry.pose_covariance.begin(), measurement.odometry.pose_covariance.end(),
          input.pose_covariance.begin(), [covariance_scale, time_diff](double x) {
            return x + x * covariance_scale * time_diff;
          });
        if (measurement.config.position_m.active)
          state_estimation_->set_input_position(input, measurement.config.position_m.num);
        if (measurement.config.orientation_rad.active)
          state_estimation_->set_input_orientation(
            input, measurement.config.orientation_rad.num,
            measurement.config.orientation_rad.mask[0],
            measurement.config.orientation_rad.mask[1],
            measurement.config.orientation_rad.mask[2]);
        if (measurement.config.linear_velocity_mps.active)
          state_estimation_->set_input_linear_velocity(
            input, measurement.config.linear_velocity_mps.num);
      } else {
        if (measurement.config.position_m.active)
          state_estimation_->set_input_position_status(
            tam::types::ErrorLvl::ERROR, measurement.config.position_m.num);
        if (measurement.config.orientation_rad.active)
          state_estimation_->set_input_orientation_status(
            tam::types::ErrorLvl::ERROR, measurement.config.orientation_rad.num);
        if (measurement.config.linear_velocity_mps.active)
          state_estimation_->set_input_linear_velocity_status(
            tam::types::ErrorLvl::ERROR, measurement.config.linear_velocity_mps.num);
      }
    }
    const auto end_delay_comp = std::chrono::high_resolution_clock::now();
    const double duration_delay_comp_us =
      std::chrono::duration_cast<std::chrono::microseconds>(end_delay_comp - start_delay_comp)
        .count();

    if (initialize_) {
      if (state_estimation_->set_initial_state()) {
        initialize_ = false;
        ROS_INFO("[StateEstimation] Initial state has been set");
        monitor_->initialization_finished();
      }
      monitor_->set_message(state_estimation_->get_state_machine_status_msg());
      ROS_INFO_THROTTLE(2.0, "[StateEstimation] waiting for initial state: %s",
        state_estimation_->get_state_machine_status_msg().c_str());
      return;
    }

    const auto start_step = std::chrono::high_resolution_clock::now();
    state_estimation_->step();
    const auto end_step = std::chrono::high_resolution_clock::now();
    const double duration_step_us =
      std::chrono::duration_cast<std::chrono::microseconds>(end_step - start_step).count();

    monitor_->set_error_lvl("internal state", state_estimation_->get_status());
    monitor_->set_message(state_estimation_->get_state_machine_status_msg());

    odometry_output_ = state_estimation_->get_odometry();
    acceleration_output_ = state_estimation_->get_acceleration();

    time_pub_ = ros::Time::now();
    time_pub_ -= ros::Duration(state_estimation_node_params_.average_input_delay_ms / 1000.0);

    const auto start_publish = std::chrono::high_resolution_clock::now();
    publish_odometry(time_pub_);
    publish_acceleration(time_pub_);
    publish_dynamic_transforms(time_pub_);
    const auto end_publish = std::chrono::high_resolution_clock::now();
    const double duration_publish_us =
      std::chrono::duration_cast<std::chrono::microseconds>(end_publish - start_publish).count();

    state_estimation_->get_state_machine_debug_output(logger_);
    state_estimation_->get_kalman_filter_debug_output(logger_);
    for (const auto& measurement : measurement_channels_) log_measurement_input(measurement);

    const ros::Time callback_time = ros::Time::now();
    const double callback_dt_us = (callback_time - last_callback_time_).toSec() * 1e6;
    last_callback_time_ = callback_time;

    logger_->log("runtime/delay_compensation", duration_delay_comp_us);
    logger_->log("runtime/step", duration_step_us);
    logger_->log("runtime/publish", duration_publish_us);
    logger_->log("runtime/callback_dt", callback_dt_us);

    const double now_sec = ros::Time::now().toSec();
    if (tsl_rate_hz_ <= 0.0 || now_sec - last_tsl_sec_ >= 1.0 / tsl_rate_hz_) {
      tsl_publisher_->trigger();
      last_tsl_sec_ = now_sec;
    }

    delay_compensation_->update_vehicle_odometry(odometry_output_);
  }

  // -------------------------------------------------------------------------
  // Publishing
  // -------------------------------------------------------------------------
  void publish_odometry(const ros::Time& time_pub)
  {
    nav_msgs::Odometry msg = tam::type_conversions::odometry_msg_from_type(odometry_output_);
    msg.header.stamp = time_pub;
    msg.header.frame_id = state_estimation_node_params_.odom_frame_id;
    msg.child_frame_id = state_estimation_node_params_.child_frame_id;
    pub_odometry_.publish(msg);
  }

  void publish_acceleration(const ros::Time& time_pub)
  {
    geometry_msgs::AccelWithCovarianceStamped msg =
      tam::type_conversions::accel_with_covariance_stamped_msg_from_type(acceleration_output_);
    msg.header.stamp = time_pub;
    msg.header.frame_id = "vehicle_cg";
    pub_acceleration_.publish(msg);
  }

  void publish_dynamic_transforms(const ros::Time& time_pub)
  {
    if (!last_tf_stamp_.isZero() && last_tf_stamp_ == time_pub) return;
    last_tf_stamp_ = time_pub;
    geometry_msgs::TransformStamped transform_local_cartesian;
    transform_local_cartesian.header.stamp = time_pub;
    transform_local_cartesian.header.frame_id = state_estimation_node_params_.odom_frame_id;
    transform_local_cartesian.child_frame_id = state_estimation_node_params_.child_frame_id;
    transform_local_cartesian.transform.translation.x = odometry_output_.position_m.x;
    transform_local_cartesian.transform.translation.y = odometry_output_.position_m.y;
    transform_local_cartesian.transform.translation.z = odometry_output_.position_m.z;
    transform_local_cartesian.transform.rotation = tam::type_conversions::euler_type_to_quaternion_msg(
      tam::types::common::EulerYPR(
        odometry_output_.orientation_rad.z, odometry_output_.orientation_rad.y,
        odometry_output_.orientation_rad.x));
    tf_broadcaster_->sendTransform(transform_local_cartesian);

    geometry_msgs::TransformStamped transform_vehicle_cg;
    transform_vehicle_cg.header.stamp = time_pub;
    transform_vehicle_cg.header.frame_id = "vehicle_cg";
    transform_vehicle_cg.child_frame_id = "vehicle_velocity_cg";
    transform_vehicle_cg.transform.translation.x = 0.0;
    transform_vehicle_cg.transform.translation.y = 0.0;
    transform_vehicle_cg.transform.translation.z = 0.0;
    transform_vehicle_cg.transform.rotation = tam::type_conversions::euler_type_to_quaternion_msg(
      tam::types::common::EulerYPR(state_estimation_->get_sideslip_angle(), 0.0, 0.0));
    tf_broadcaster_->sendTransform(transform_vehicle_cg);
  }

  void log_measurement_input(const tam::types::state::measurement& measurement)
  {
    if (!measurement.config.position_m.active || !measurement.config.orientation_rad.active) return;
    const std::string debug_namespace = "pose/" + measurement.config.name;
    auto log_odometry = [&](const std::string& prefix,
                            const tam::types::control::Odometry& odom, double delay) {
      logger_->log(debug_namespace + prefix + "/position_m/x", odom.position_m.x);
      logger_->log(debug_namespace + prefix + "/position_m/y", odom.position_m.y);
      logger_->log(debug_namespace + prefix + "/position_m/z", odom.position_m.z);
      logger_->log(debug_namespace + prefix + "/orientation_rad/roll", odom.orientation_rad.x);
      logger_->log(debug_namespace + prefix + "/orientation_rad/pitch", odom.orientation_rad.y);
      logger_->log(debug_namespace + prefix + "/orientation_rad/yaw", odom.orientation_rad.z);
      logger_->log(debug_namespace + prefix + "/velocity_mps/x", odom.velocity_mps.x);
      logger_->log(debug_namespace + prefix + "/velocity_mps/y", odom.velocity_mps.y);
      logger_->log(debug_namespace + prefix + "/velocity_mps/z", odom.velocity_mps.z);
      logger_->log(debug_namespace + prefix + "/msg_delay", delay);
    };
    if (measurement.received) {
      tam::types::control::Odometry delay_compensated = measurement.odometry;
      const double measured_time_diff =
        (measurement.received_stamp - measurement.measurement_stamp).toSec();
      delay_compensation_->compensate(delay_compensated, measured_time_diff);
      log_odometry("/measured", delay_compensated, measured_time_diff);

      tam::types::control::Odometry virtual_odometry = measurement.odometry;
      const double virtual_time_diff = (ros::Time::now() - measurement.measurement_stamp).toSec();
      delay_compensation_->compensate(virtual_odometry, virtual_time_diff);
      log_odometry("/virtual", virtual_odometry, virtual_time_diff);
    } else {
      log_odometry("/measured", {}, 0.0);
      log_odometry("/virtual", {}, 0.0);
    }
  }

  // -------------------------------------------------------------------------
  // Callbacks
  // -------------------------------------------------------------------------
  void measurement_callback(
    const nav_msgs::Odometry::ConstPtr& msg, tam::types::state::measurement& measurement)
  {
    watchdog_.update(measurement.config.measurement_sub.topic);
    ROS_INFO_THROTTLE(5.0, "[StateEstimation] measurement received on %s (stamp %.3f)",
      measurement.config.measurement_sub.topic.c_str(), msg->header.stamp.toSec());
    measurement.odometry = tam::type_conversions::odometry_type_from_msg(*msg);

    if (measurement.config.position_m.active || measurement.config.orientation_rad.active) {
      try {
        if (!measurement.static_translation_valid) {
          geometry_msgs::TransformStamped transform = tf_buffer_->lookupTransform(
            msg->child_frame_id, state_estimation_node_params_.child_frame_id, ros::Time(0));
          measurement.static_translation = tf2::Vector3(
            transform.transform.translation.x, transform.transform.translation.y,
            transform.transform.translation.z);
          measurement.static_translation_valid = true;
        }
        tf2::Matrix3x3 rotation_matrix;
        if (initialize_) {
          rotation_matrix.setEulerYPR(
            measurement.odometry.orientation_rad.z, measurement.odometry.orientation_rad.y,
            measurement.odometry.orientation_rad.x);
        } else {
          rotation_matrix.setEulerYPR(
            odometry_output_.orientation_rad.z, odometry_output_.orientation_rad.y,
            odometry_output_.orientation_rad.x);
        }
        const tf2::Vector3 transformation = rotation_matrix * measurement.static_translation;
        measurement.odometry.position_m = measurement.odometry.position_m +
                                          tam::types::common::Vector3D<double>{
                                            transformation.x(), transformation.y(),
                                            transformation.z()};
      } catch (const tf2::TransformException& ex) {
        ROS_INFO("[StateEstimation] Failed to transform: %s", ex.what());
        return;
      }
    }

    ros::Time measurement_timestamp = msg->header.stamp;
    const auto additional_delay = ros::Duration(measurement.config.additional_delay / 1000.0);
    const auto average_delay =
      ros::Duration(state_estimation_node_params_.average_input_delay_ms / 1000.0);
    if ((measurement_timestamp - additional_delay + average_delay) <= ros::Time::now()) {
      measurement_timestamp -= additional_delay - average_delay;
    } else {
      measurement_timestamp -= additional_delay;
    }

    measurement.measurement_stamp = measurement_timestamp;
    measurement.received_stamp = ros::Time::now();
    measurement.received = true;
  }

  void measurement_status_callback(
    const diagnostic_msgs::DiagnosticArray::ConstPtr& msg,
    const tam::types::state::measurement& measurement)
  {
    watchdog_.update(measurement.config.status_sub.topic);
    if (msg->status.empty()) return;
    ROS_INFO_THROTTLE(5.0, "[StateEstimation] status level %d (%s) on %s",
      static_cast<int>(msg->status[0].level), msg->status[0].message.c_str(),
      measurement.config.status_sub.topic.c_str());
    if (measurement.config.position_m.active)
      state_estimation_->set_input_position_status(
        tam::type_conversions::error_type_from_diagnostic_level(msg->status[0].level),
        measurement.config.position_m.num);
    if (measurement.config.orientation_rad.active)
      state_estimation_->set_input_orientation_status(
        tam::type_conversions::error_type_from_diagnostic_level(msg->status[0].level),
        measurement.config.orientation_rad.num);
    if (measurement.config.linear_velocity_mps.active)
      state_estimation_->set_input_linear_velocity_status(
        tam::type_conversions::error_type_from_diagnostic_level(msg->status[0].level),
        measurement.config.linear_velocity_mps.num);
  }

  void input_callback(const sensor_msgs::Imu::ConstPtr& msg, tam::types::state::input& input)
  {
    watchdog_.update(input.config.measurement_sub.topic);
    const auto acceleration =
      tam::type_conversions::acceleration_with_covariances_type_from_imu_msg(*msg);
    const auto odometry = tam::type_conversions::odometry_type_from_imu_msg(*msg);
    state_estimation_->set_input_imu_status(tam::types::ErrorLvl::OK, input.config.num);
    state_estimation_->set_input_acceleration(acceleration, input.config.num);
    state_estimation_->set_input_angular_velocity(odometry, input.config.num);
    input.received = true;
  }

  void measurement_timeout_callback(
    bool timeout, const tam::types::state::measurement& measurement)
  {
    if (!timeout) return;
    if (measurement.config.position_m.active)
      state_estimation_->set_position_timeout(measurement.config.position_m.num);
    if (measurement.config.orientation_rad.active)
      state_estimation_->set_orientation_timeout(measurement.config.orientation_rad.num);
    if (measurement.config.linear_velocity_mps.active)
      state_estimation_->set_linear_velocity_timeout(measurement.config.linear_velocity_mps.num);
  }

  void imu_timeout_callback(bool timeout, const tam::types::state::input& input)
  {
    if (timeout && input.received) state_estimation_->set_imu_timeout(input.config.num);
  }

  // -------------------------------------------------------------------------
  // Members
  // -------------------------------------------------------------------------
  ros::NodeHandle nh_, pnh_;
  std::unique_ptr<tam::core::state::StateEstimationBase> state_estimation_{};
  tam::pmg::ParamManagerComposer::SharedPtr param_manager_composer_{};
  tam::pmg::ParamValueManager::SharedPtr param_manager_ =
    std::make_shared<tam::pmg::ParamValueManager>();

  std::size_t previous_param_state_hash_{0};
  tam::types::state::config::node state_estimation_node_params_{};
  ros::Timer timer_{};
  ros::Time time_pub_{};
  ros::Time last_tf_stamp_{};
  ros::Time last_callback_time_{};
  double last_tsl_sec_{0.0};
  double tsl_rate_hz_{10.0};
  bool initialize_{true};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_{};
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_{};

  std::unique_ptr<tam::core::NodeMonitor> monitor_{};
  tam::core::TopicWatchdog watchdog_{};
  std::unique_ptr<tam::helpers::DelayCompensation> delay_compensation_{};

  std::vector<ros::Subscriber> measurement_subs_, status_subs_, imu_subs_;
  ros::Subscriber wheel_sub_, wheel_status_sub_, steering_sub_;

  ros::Publisher pub_odometry_, pub_acceleration_;

  tam::types::control::Odometry odometry_output_{};
  tam::types::control::AccelerationwithCovariances acceleration_output_{};

  tam::tsl::ValueLogger::SharedPtr logger_ = std::make_shared<tam::tsl::ValueLogger>();
  std::unique_ptr<TslPublisher> tsl_publisher_{};

  std::vector<tam::types::state::measurement> measurement_channels_{};
  std::vector<tam::types::state::input> input_channels_{};
};

}  // namespace farlio_ros1
