// Copyright 2024 Maximilian Leitenstern

#pragma once
// https://pcl.readthedocs.io/projects/tutorials/en/latest/adding_custom_ptype.html#how-to-add-a-new-pointt-type
#define PCL_NO_PRECOMPILE
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
namespace tam::types::perception
{
/********************************************************
 * @brief LIDARPoint
 * @brief define custom point type containing lidar data
 * @note id can be used to store sensor id in multi-sensor setups
 ********************************************************/
struct EIGEN_ALIGN16 LIDARPoint  // enforce SSE padding for correct memory alignment
{
  PCL_ADD_POINT4D;  // preferred way of adding a XYZ+padding
  float intensity;
  double timestamp;
  uint8_t id;
  PCL_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
};
/********************************************************
 * @brief RADARPoint
 * @brief define custom point type containing radar data
 ********************************************************/
struct EIGEN_ALIGN16 RADARPoint
{
  PCL_ADD_POINT4D;
  float range;      // originally captured, not to be transformed
  float azimuth;    // originally captured, not to be transformed
  float elevation;  // originally captured, not to be transformed
  float velocity;
  float snr;
  float rcs;
  float confidence;
  float velocity_interval;
  uint8_t id;
  PCL_MAKE_ALIGNED_OPERATOR_NEW
};
/********************************************************
 * @brief XYZISPoint
 * @brief define custom point type containing segment id
 * @note segment to be used for semantic segmentation labels
 ********************************************************/
struct EIGEN_ALIGN16 XYZISPoint  // enforce SSE padding for correct memory alignment
{
  PCL_ADD_POINT4D;  // preferred way of adding a XYZ+padding
  float intensity;
  uint32_t segment;
  PCL_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
};
/********************************************************
 * @brief XYZITAEDPoint
 * @brief define custom point type containing timestamp and additional lidar data
 ********************************************************/
struct EIGEN_ALIGN16 XYZITAEDPoint
{
  PCL_ADD_POINT4D;  // quad-word XYZ
  double timestamp;
  float intensity;
  float azimuth;
  float elevation;
  float distance;
  PCL_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace tam::types::perception

POINT_CLOUD_REGISTER_POINT_STRUCT(
  tam::types::perception::LIDARPoint,
  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
    double, timestamp, timestamp)(uint8_t, id, id))
POINT_CLOUD_REGISTER_POINT_STRUCT(
  tam::types::perception::RADARPoint,
  (float, x, x)(float, y, y)(float, z, z)(float, range, range)(float, azimuth, azimuth)(
    float, elevation, elevation)(float, velocity, velocity)(float, snr, snr)(float, rcs, rcs)(
    float, confidence, confidence)(float, velocity_interval, velocity_interval)(uint8_t, id, id))
POINT_CLOUD_REGISTER_POINT_STRUCT(
  tam::types::perception::XYZISPoint,
  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint32_t, segment, segment))
POINT_CLOUD_REGISTER_POINT_STRUCT(
  tam::types::perception::XYZITAEDPoint,
  (float, x,
   x)(float, y, y)(float, z, z)(double, timestamp, timestamp)(float, intensity, intensity)(
    float, azimuth, azimuth)(float, elevation, elevation)(float, distance, distance))
