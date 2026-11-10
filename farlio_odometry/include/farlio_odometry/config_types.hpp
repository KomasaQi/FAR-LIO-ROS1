// Licensed under the Apache License, Version 2.0.
#pragma once

#include "odometry_types/odometry_config.hpp"

namespace farlio_ros1 {
/**
 * @brief GICP + constant-velocity model (upstream only ships GICP_EXT / ICP_CV).
 *
 * Identical to tam::core::state::types::GICP_EXT but with an internal constant
 * velocity prediction, i.e. standalone LiDAR-inertial-free odometry mode.
 */
struct GICP_CV {
  static constexpr bool SPHERICAL = false;
  static constexpr bool INTENSITY = false;
  static constexpr bool NORMALS = true;
  static constexpr bool COV = true;
  static constexpr bool SEG = false;
  static constexpr bool VEL = false;
  static constexpr bool RCS = false;
  static constexpr bool SNR = false;
  static constexpr bool CONFIDENCE = false;
  static constexpr bool VEL_INTERVAL = false;
  static constexpr bool SENSOR_ID = false;
  static constexpr int16_t MAX_POINTS_PER_VOXEL = 40;
  static constexpr int16_t NUM_NEIGHBORS = 10;
  static constexpr tam::core::state::types::CovarianceType COVARIANCE = tam::core::state::types::CovarianceType::CONSTANT;
  static constexpr tam::core::state::types::DiagnosticType DIAGNOSTIC = tam::core::state::types::DiagnosticType::BASE;
  static constexpr tam::core::state::types::DistortionType DISTORTION = tam::core::state::types::DistortionType::POLYNOM;
  static constexpr tam::core::state::types::RobustKernelType KERNEL = tam::core::state::types::RobustKernelType::CAUCHY;
  static constexpr tam::core::state::types::MapType MAP = tam::core::state::types::MapType::VOXELHASHMAP;
  static constexpr tam::core::state::types::ModelType MODEL = tam::core::state::types::ModelType::CONSTANTVELOCITY;
  static constexpr tam::core::state::types::PreprocessingType PREPROCESSING = tam::core::state::types::PreprocessingType::LIDAR;
  static constexpr tam::core::state::types::RegistrationType REGISTRATION = tam::core::state::types::RegistrationType::GICP;
  static constexpr tam::core::state::types::ThresholdType THRESHOLD = tam::core::state::types::ThresholdType::ADAPTIVETHRESHOLD;
  static constexpr tam::core::state::types::VelocityType VELOCITY = tam::core::state::types::VelocityType::DERIVATIVE;
};
}  // namespace farlio_ros1
