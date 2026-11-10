// ROS 1 shim for TUMFTM/TAM__common :: tum_type_conversions_ros_cpp
//
// The FAR-LIO state-estimation library only needs the diagnostic-level mapping
// from that package (everything else is ROS-message conversion, which this port
// implements natively for ROS 1 in farlio_state_estimation/ros1_type_conversions.hpp).
//
// The numeric values of tam::types::ErrorLvl and diagnostic_msgs::DiagnosticStatus
// are identical in ROS 1 (OK=0, WARN=1, ERROR=2, STALE=3).
#pragma once

#include <iostream>

#include "tum_types_cpp/common.hpp"

namespace tam::type_conversions {
inline unsigned char diagnostic_level_from_type(const tam::types::ErrorLvl& lvl)
{
  switch (lvl) {
    case tam::types::ErrorLvl::OK:
      return 0;
    case tam::types::ErrorLvl::WARN:
      return 1;
    case tam::types::ErrorLvl::ERROR:
      return 2;
    case tam::types::ErrorLvl::STALE:
      return 3;
    default:
      std::cerr << "Unknown Error Level\n";
      return 2;
  }
}

inline tam::types::ErrorLvl error_type_from_diagnostic_level(unsigned char lvl)
{
  switch (lvl) {
    case 0:
      return tam::types::ErrorLvl::OK;
    case 1:
      return tam::types::ErrorLvl::WARN;
    case 2:
      return tam::types::ErrorLvl::ERROR;
    case 3:
      return tam::types::ErrorLvl::STALE;
    default:
      std::cerr << "Unknown Error Level\n";
      return tam::types::ErrorLvl::ERROR;
  }
}
}  // namespace tam::type_conversions
