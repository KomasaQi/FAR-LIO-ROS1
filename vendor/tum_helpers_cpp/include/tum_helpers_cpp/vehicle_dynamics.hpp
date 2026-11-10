// Copyright 2024 Maximilian Leitenstern
#pragma once

#include <algorithm>
#include <eigen3/Eigen/Dense>

#include "tum_helpers_cpp/aerodynamics.hpp"
#include "tum_helpers_cpp/constants.hpp"
#include "tum_helpers_cpp/numerical.hpp"
#include "tum_types_cpp/control.hpp"
#include "tum_types_cpp/data_per_wheel.hpp"
#include "tum_types_cpp/tire.hpp"
#include "tum_types_cpp/vehicle.hpp"
namespace tam::helpers::vehicle_dynamics
{
/**
 * @brief Definition for dynamic tire radius
 * @param vel Velocity over ground of the tire
 * @param params Parameters of the four tires
 * @return tam::types::common::DataPerWheel<double> Dynamic tire radii
 */
tam::types::common::DataPerWheel<double> dynamic_tire_radius(
  const double vel_x,
  const tam::types::common::DataPerWheel<tam::types::tire_params::Tire> & params);

/**
 * @brief Calculate slip ratios of the vehicle
 * @param steering_angle Steering angle of the vehicle [rad]
 * @param wheelspeeds Rotational velocity of the wheels [rad/s]
 * @param tire_params Parameters of the tir
 * @param params Parameter of car dimensions
 * @return tam::types::common::DataPerWheel<double>
 */
tam::types::common::DataPerWheel<double> long_slip(
  const tam::types::control::Odometry & odom, const double steering_angle,
  const tam::types::common::DataPerWheel<double> & wheelspeeds,
  const tam::types::common::DataPerWheel<tam::types::tire_params::Tire> & tire_params,
  const tam::types::vehicle_params::Dimension & params);

/**
 * @brief Calculate slip angles of the vehicle
 * @param odom pose of vehicle
 * @param steering_angle steering angle of vehicle
 * @param params_vehicle vehicle parameters for dimension
 * @param params_tire tire parameters
 * @return tam::types::common::DataPerWheel<double>
 */
tam::types::common::DataPerWheel<double> slip_angle(
  const tam::types::control::Odometry & odom, const double steering_angle,
  const tam::types::vehicle_params::Dimension & params_vehicle,
  const tam::types::common::DataPerWheel<tam::types::tire_params::Tire> & params_tire);

/**
 * @brief Calculate the vertical load on each tire.
 * @param velocity_mps Vehicle velocity in m/s as a 3D vector.
 * @param ax_mps2 Longitudinal acceleration in m/s².
 * @param ay_mps2 Lateral acceleration in m/s².
 * @param vehicle_ Vehicle parameters.
 * @return tam::types::common::DataPerWheel<double> Computed vertical load per tire.
 */
tam::types::common::DataPerWheel<double> tire_vertical_load(
  const tam::types::common::Vector3D<double> & velocity_mps, double ax_mps2, double ay_mps2,
  const tam::types::vehicle_params::Vehicle & vehicle_);
}  // namespace tam::helpers::vehicle_dynamics
