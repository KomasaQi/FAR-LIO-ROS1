// Licensed under the Apache License, Version 2.0.
//
// Explicit instantiation of the FAR-LIO pipeline for the GICP + constant-velocity
// configuration used by the ROS 1 standalone odometry mode. Upstream only
// instantiates ICP_EXT, ICP_CV and GICP_EXT.
#include "odometry_pipeline/odometry_pipeline_impl.hpp"

#include "farlio_odometry/config_types.hpp"

namespace tam::core::state {
template class OdometryPipeline<farlio_ros1::GICP_CV>;
}  // namespace tam::core::state
