// Licensed under the Apache License, Version 2.0.
// FAR-LIO state estimation variant: EKF_3D + SingleTrack.
#include "farlio_odometry/estimation/main_impl.hpp"

int main(int argc, char** argv)
{
  return farlio_ros1::run_state_estimation<tam::core::state::EKF_3D,
    tam::core::state::VehicleModel::SingleTrack>(argc, argv, "state_estimation_3d_stm");
}
