// Copyright 2024 Daniel Esser
#pragma once
#include "tum_types_cpp/common.hpp"
namespace tam::types::prediction
{
// Inputs
using ObjectID = std::array<uint8_t, 16>;
struct ObjectIDHash
{
  std::size_t operator()(const ObjectID & id) const
  {
    std::size_t seed = 0;
    for (const uint8_t byte : id) {
      seed ^= byte + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};
struct TrackedObject
{
  ObjectID object_id;
  /// @brief range (min=0.0, max=1.0)
  float existence_probability;
  bool is_stationary;
  mutable bool is_in_pit = false;
  tam::types::common::Vector3D<double> position_m;
  tam::types::common::Vector3D<double>
    orientation_rad;  // Euler Angles - therefore the assignement is (x:roll, y:pitch, z:yaw)
  /*6x6 covariance matrix
  The orientation parameters use a fixed-axis representation.
  In order, the parameters are:
  (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis) */
  std::array<double, 36> pose_covariance;  //  Row Major Matrix Representation

  tam::types::common::Vector3D<double>
    velocity_mps;  // In the "local" coordinate system speficied by position and orientation
  tam::types::common::Vector3D<double>
    angular_velocity_radps;  // In the "local" coordinate system speficied by
                             // position and orientation
  /*6x6 covariance matrix
  The orientation parameters use a fixed-axis representation.
  In order, the parameters are:
  (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis) */
  std::array<double, 36> velocity_covariance;  //  Row Major Matrix Representation
  tam::types::common::Vector3D<double>
    acceleration_mps2;  // In the "local" coordinate system speficied by position and orientation
  tam::types::common::Vector3D<double>
    angular_acceleration_radps2;  // In the "local" coordinate system speficied by
                                  // position and orientation
  /*6x6 covariance matrix
  The orientation parameters use a fixed-axis representation.
  In order, the parameters are:
  (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis) */
  std::array<double, 36> acceleration_covariance;  //  Row Major Matrix Representation
};
struct TrackedObjects
{
  tam::types::common::Header header;
  std::vector<TrackedObject> objects;
};
using TrackedObjectCref = std::reference_wrapper<const TrackedObject>;
//
// Outputs
struct Pose
{
  tam::types::common::Vector3D<double> position_m;
  tam::types::common::Vector3D<double> orientation_rad;  // Euler Angles - therefore the assignement
                                                         // is (x:roll, y:pitch, z:yaw)
  /*6x6 covariance matrix
  The orientation parameters use a fixed-axis representation.
  In order, the parameters are:
  (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis) */
};
struct PredictedPath
{
  float confidence;
  float time_step_s;
  std::vector<Pose> path;
};
template <typename TrackedObjectType>
struct PredictedObject_
{
  TrackedObjectType tracked_object;
  std::vector<PredictedPath> predicted_paths{};
};
using PredictedObject = PredictedObject_<TrackedObjectCref>;
using PredictedObjectWithOwnedTrackedObject = PredictedObject_<TrackedObject>;
struct PredictedObjects
{
  tam::types::common::Header header;
  std::vector<PredictedObject> objects;
};
struct PredictedObjectsWithOwnedTrackedObjects
{
  tam::types::common::Header header;
  std::vector<PredictedObjectWithOwnedTrackedObject> objects;
};
}  // namespace tam::types::prediction
