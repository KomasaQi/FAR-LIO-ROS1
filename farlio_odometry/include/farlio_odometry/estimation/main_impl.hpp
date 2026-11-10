// Licensed under the Apache License, Version 2.0.
//
// Shared entry point for the six FAR-LIO state-estimation variants
// (2D/3D EKF x Kinematic/NonHolonomic/SingleTrack), mirroring upstream's
// six tam_state_estimation_node executables.
#pragma once

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "farlio_odometry/estimation/node.hpp"
#include "farlio_odometry/yaml_params.hpp"
#include "param_management_cpp/initialization.hpp"
#include "state_estimation_base/state_estimation_base.hpp"
#include "state_estimation_constants/EKF_2D.hpp"
#include "state_estimation_constants/EKF_3D.hpp"
#include "tam_state_estimation/state_estimation.hpp"

namespace farlio_ros1 {

template <class TConfig, tam::core::state::VehicleModel TModel>
int run_state_estimation(int argc, char** argv, const std::string& node_name)
{
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string config_file;
  if (!pnh.getParam("config_file", config_file)) {
    ROS_FATAL("[StateEstimation] ~config_file is required.");
    return 1;
  }

  YAML::Node config;
  try {
    config = YAML::LoadFile(config_file);
  } catch (const std::exception& e) {
    ROS_FATAL("[StateEstimation] Cannot load '%s': %s", config_file.c_str(), e.what());
    return 1;
  }

  // Feed the YAML values into the ROS-free parameter managers before the
  // estimator declares its parameters (equivalent to ROS 2 parameter overrides).
  std::vector<std::pair<std::string, tam::pmg::param_value_variant_t>> params;
  flatten_yaml(config, "", params);
  tam::pmg::init(params);

  try {
    auto estimator =
      std::make_unique<tam::core::state::StateEstimation<TConfig, TModel>>();
    StateEstimationNode<TConfig, TModel> node(nh, pnh, std::move(estimator), config);
    ros::spin();
  } catch (const std::exception& e) {
    ROS_FATAL("[StateEstimation] %s", e.what());
    return 1;
  }
  return 0;
}

}  // namespace farlio_ros1
