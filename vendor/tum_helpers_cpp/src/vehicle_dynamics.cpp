// Copyright 2024 Maximilian Leitenstern

#include "tum_helpers_cpp/vehicle_dynamics.hpp"
namespace tam::helpers::vehicle_dynamics
{
/******************************************************
 * @brief Static helper functions for vehicle dynamics
 ******************************************************/
/**
 * @brief Definition for longitudinal slip
 * @param rot_vel Rotational velocity of the wheel
 * @param vel_over_ground Velocity over ground of tire
 * @return double slip ratio
 * @note -> slip ratio is clamped between -1 and 1
 * @note -> slip ratio is calculated as (rot_vel - vel_over_ground) / max(3.0, vel_over_ground)
 * @note -> i.e. vel over ground is bounded by 3.0 m/s in the denominator to avoid numverical issues
 */
static double slip_def(const double rot_vel, const double vel_over_ground)
{
  return std::clamp((rot_vel - vel_over_ground) / std::max(3.0, vel_over_ground), -1.0, 1.0);
}
/**
 * @brief Definition for lateral slip angle of tire
 * @param vx Velocity in x direction
 * @param vy Velocity in y direction
 * @return double slip angle
 * @note -> slip angle is calculated as -atan2(vy, max(1.0, vx))
 * @note -> vx bounded by 1.0 to avoid numerical issues
 */
static double slip_angle_def(const double vx, const double vy)
{
  return -std::atan2(vy, std::max(1.0, vx));
}
/**
 * @brief Definition for dynamic tire radius
 * @param vel Velocity over ground of the tire
 * @param params radius related parameters of the tire
 * @return double Dynamic tire radius (interpolated)
 * @note -> dynamic tire radius interpolated based on lookup table
 */
static double dynamic_tire_radius(
  const double vel_x, const tam::types::tire_params::Radius & params)
{
  return params.radius_20mps *
         tam::helpers::numerical::interp(
           vel_x, params.velocity_scaling.velocity, params.velocity_scaling.factor);
}
/**
 * @brief Calculate velocity vectors of wheels from COG pose in vehicle frame
 * @param odom pose of vehicle
 * @param params vehicle parameters for dimension
 * @return tam::types::common::DataPerWheel<tam::types::common::Vector2D<double>>
 */
static tam::types::common::DataPerWheel<tam::types::common::Vector2D<double>>
velocity_wheel_over_ground(
  const tam::types::control::Odometry & odom, const tam::types::vehicle_params::Dimension & params)
{
  tam::types::common::DataPerWheel<tam::types::common::Vector2D<double>> vel({0.0, 0.0});
  vel.front_left = {
    odom.velocity_mps.x - params.track_width_front * odom.angular_velocity_radps.z / 2,
    odom.velocity_mps.y + params.distance_to_front_axle * odom.angular_velocity_radps.z};
  vel.front_right = {
    odom.velocity_mps.x + params.track_width_front * odom.angular_velocity_radps.z / 2,
    odom.velocity_mps.y + params.distance_to_front_axle * odom.angular_velocity_radps.z};
  vel.rear_left = {
    odom.velocity_mps.x - params.track_width_rear * odom.angular_velocity_radps.z / 2,
    odom.velocity_mps.y -
      (params.wheelbase - params.distance_to_front_axle) * odom.angular_velocity_radps.z};
  vel.rear_right = {
    odom.velocity_mps.x + params.track_width_rear * odom.angular_velocity_radps.z / 2,
    odom.velocity_mps.y -
      (params.wheelbase - params.distance_to_front_axle) * odom.angular_velocity_radps.z};
  return vel;
}
/**
 * @brief Calculate velocity vectors of wheels from vehicle coordinate frame to TRUE tire frame
 * @param steering_angle Steering angle of the vehicle
 * @param velocity_over_ground_vehicle_frame velocity of tires in vehicle frame
 * @param params_tire Tire parameters including static toe angles
 * @return tam::types::common::DataPerWheel<tam::types::common::Vector2D<double>> Velocity in true
 * tire frame
 * @note This version rotates each wheel's velocity into its specific frame defined
 *       by steering (front only) AND static toe angle.
 */
static tam::types::common::DataPerWheel<tam::types::common::Vector2D<double>>
velocity_wheel_over_ground_tire_frame(
  const double steering_angle,
  const tam::types::common::DataPerWheel<tam::types::common::Vector2D<double>> &
    velocity_over_ground_vehicle_frame,
  const tam::types::common::DataPerWheel<tam::types::tire_params::Tire> & params_tire)
{
  tam::types::common::DataPerWheel<tam::types::common::Vector2D<double>> vel_true_tire_frame(
    {0.0, 0.0});

  // --- Calculate total rotation angles relative to vehicle frame ---
  // Assuming positive toe is toe-out.
  const double total_angle_fl =
    steering_angle + params_tire.front_left.setup.toe;  // Toe-out increases angle on left
  const double total_angle_fr = steering_angle - params_tire.front_right.setup.toe;
  const double total_angle_rl = params_tire.rear_left.setup.toe;  // Toe-out increased angle on left
  const double total_angle_rr = -params_tire.rear_right.setup.toe;

  // --- Pre-calculate trig functions ---
  const double cos_fl = std::cos(total_angle_fl);
  const double sin_fl = std::sin(total_angle_fl);
  const double cos_fr = std::cos(total_angle_fr);
  const double sin_fr = std::sin(total_angle_fr);
  const double cos_rl = std::cos(total_angle_rl);
  const double sin_rl = std::sin(total_angle_rl);
  const double cos_rr = std::cos(total_angle_rr);
  const double sin_rr = std::sin(total_angle_rr);

  // --- Perform rotation for each wheel ---
  // Rotation Matrix: [[cos, sin], [-sin, cos]] transforms vector INTO the new frame

  const auto & vel_veh_fl = velocity_over_ground_vehicle_frame.front_left;
  vel_true_tire_frame.front_left = {
    vel_veh_fl.x * cos_fl + vel_veh_fl.y * sin_fl,  // x-component in tire frame
    -vel_veh_fl.x * sin_fl + vel_veh_fl.y * cos_fl  // y-component in tire frame
  };

  const auto & vel_veh_fr = velocity_over_ground_vehicle_frame.front_right;
  vel_true_tire_frame.front_right = {
    vel_veh_fr.x * cos_fr + vel_veh_fr.y * sin_fr, -vel_veh_fr.x * sin_fr + vel_veh_fr.y * cos_fr};

  const auto & vel_veh_rl = velocity_over_ground_vehicle_frame.rear_left;
  vel_true_tire_frame.rear_left = {
    vel_veh_rl.x * cos_rl + vel_veh_rl.y * sin_rl, -vel_veh_rl.x * sin_rl + vel_veh_rl.y * cos_rl};

  const auto & vel_veh_rr = velocity_over_ground_vehicle_frame.rear_right;
  vel_true_tire_frame.rear_right = {
    vel_veh_rr.x * cos_rr + vel_veh_rr.y * sin_rr, -vel_veh_rr.x * sin_rr + vel_veh_rr.y * cos_rr};

  return vel_true_tire_frame;
}
/**
 * @brief Calculate the rotational velocity of the tire
 * @param vel_x Velocity over ground of the tire
 * @param wheelspeeds Rotational velocity of the wheels
 * @param params Parameters of the tire
 * @return tam::types::common::DataPerWheel<double>
 */
static tam::types::common::DataPerWheel<double> rot_velocity_tire(
  const double vel_x, const tam::types::common::DataPerWheel<double> & wheelspeeds,
  const tam::types::common::DataPerWheel<tam::types::tire_params::Tire> & params)
{
  return wheelspeeds * dynamic_tire_radius(vel_x, params);
}
/******************************************************
 * @brief Public implementations for header functions
 ******************************************************/
tam::types::common::DataPerWheel<double> dynamic_tire_radius(
  const double vel_x,
  const tam::types::common::DataPerWheel<tam::types::tire_params::Tire> & params)
{
  return tam::types::common::DataPerWheel<double>(
    dynamic_tire_radius(vel_x, params.front_left.radius),
    dynamic_tire_radius(vel_x, params.front_right.radius),
    dynamic_tire_radius(vel_x, params.rear_left.radius),
    dynamic_tire_radius(vel_x, params.rear_right.radius));
}
tam::types::common::DataPerWheel<double> long_slip(
  const tam::types::control::Odometry & odom, const double steering_angle,
  const tam::types::common::DataPerWheel<double> & wheelspeeds,
  const tam::types::common::DataPerWheel<tam::types::tire_params::Tire> & tire_params,
  const tam::types::vehicle_params::Dimension & params)
{
  // Calculate wheel velocities from COG pose in vehicle frame
  tam::types::common::DataPerWheel<tam::types::common::Vector2D<double>> vel_wheel =
    velocity_wheel_over_ground(odom, params);
  // Convert vehice frame to tire frame using steering angle
  tam::types::common::DataPerWheel<tam::types::common::Vector2D<double>> vel_tire =
    velocity_wheel_over_ground_tire_frame(steering_angle, vel_wheel, tire_params);

  tam::types::common::DataPerWheel<double> rot_vel_tire =
    rot_velocity_tire(odom.velocity_mps.x, wheelspeeds, tire_params);

  tam::types::common::DataPerWheel<double> slip;
  slip.front_left = slip_def(rot_vel_tire.front_left, vel_tire.front_left.x);
  slip.front_right = slip_def(rot_vel_tire.front_right, vel_tire.front_right.x);
  slip.rear_left = slip_def(rot_vel_tire.rear_left, vel_tire.rear_left.x);
  slip.rear_right = slip_def(rot_vel_tire.rear_right, vel_tire.rear_right.x);

  return slip;
}
tam::types::common::DataPerWheel<double> slip_angle(
  const tam::types::control::Odometry & odom, const double steering_angle,
  const tam::types::vehicle_params::Dimension & params_vehicle,
  const tam::types::common::DataPerWheel<tam::types::tire_params::Tire> & params_tire)
{
  // clang-format off
  // Calculate wheel velocities from COG pose in vehicle frame
  tam::types::common::DataPerWheel<tam::types::common::Vector2D<double>> vel_wheel = velocity_wheel_over_ground(odom, params_vehicle);  // NOLINT
  // Convert vehicle frame velocity to TRUE tire frame velocity (accounts for steering AND toe)
  // NOTE: Now passing params_tire to this helper function
  tam::types::common::DataPerWheel<tam::types::common::Vector2D<double>> vel_tire = velocity_wheel_over_ground_tire_frame(steering_angle, vel_wheel, params_tire); // NOLINT
  // Calculate slip angles
  tam::types::common::DataPerWheel<double> slip;
  slip.front_left = slip_angle_def(vel_tire.front_left.x, vel_tire.front_left.y);  // NOLINT
  slip.front_right = slip_angle_def(vel_tire.front_right.x, vel_tire.front_right.y);  // NOLINT
  slip.rear_left = slip_angle_def(vel_tire.rear_left.x, vel_tire.rear_left.y);  // NOLINT
  slip.rear_right = slip_angle_def(vel_tire.rear_right.x, vel_tire.rear_right.y);  // NOLINT
  // clang-format on
  return slip;
}
tam::types::common::DataPerWheel<double> tire_vertical_load(
  const tam::types::common::Vector3D<double> & velocity_mps, double ax_mps2, double ay_mps2,
  const tam::types::vehicle_params::Vehicle & vehicle)
{
  constexpr float G_CONST = tam::constants::g_earth;
  tam::types::common::DataPerWheel<double> fz_tire_N;

  // Derived parameter: distance from CoG to rear axle [m]
  const double l_r_cog = vehicle.dimension.wheelbase - vehicle.dimension.distance_to_front_axle;

  // --- 1. Static load distribution (positive downwards) ---
  // Caller must ensure vehicle.dimension.wheelbase is not zero.
  const double fz_static_f_N = vehicle.mass.total * G_CONST * l_r_cog / vehicle.dimension.wheelbase;
  const double fz_static_r_N =
    vehicle.mass.total * G_CONST * vehicle.dimension.distance_to_front_axle /
    vehicle.dimension.wheelbase;

  // --- 2. Aerodynamic load component ---
  const tam::types::vehicle_params::AeroModelOutput aero_output =
    tam::helpers::aerodynamics::eval_model(velocity_mps, vehicle.aero);
  const double fz_aero_total_N =
    -aero_output.force_cog.z;  // Total aero downforce (Z, flip sign to make downforce positive)
  const double aero_torque_y =
    aero_output.torque
      .y;  // Aero pitching moment about CoG Y-axis (positive = pitch-up, lifts nose)

  // Distribute total aerodynamic force and moment to axles.
  // Caller must ensure vehicle.dimension.wheelbase is not zero.
  const double dfz_aero_f_N =
    (fz_aero_total_N * l_r_cog - aero_torque_y) / vehicle.dimension.wheelbase;
  const double dfz_aero_r_N =
    (fz_aero_total_N * vehicle.dimension.distance_to_front_axle + aero_torque_y) /
    vehicle.dimension.wheelbase;

  // --- 3. Longitudinal load transfer component ---
  // Positive ax_mps2 (forward accel) transfers load from front to rear.
  // Caller must ensure vehicle.dimension.wheelbase is not zero.
  const double dfz_long_transfer_N =
    vehicle.mass.total * ax_mps2 * vehicle.dimension.cog_height / vehicle.dimension.wheelbase;

  // --- Total estimated vertical loads per axle (positive downwards) ---
  const double fz_f_N = fz_static_f_N + dfz_aero_f_N - dfz_long_transfer_N;
  const double fz_r_N = fz_static_r_N + dfz_aero_r_N + dfz_long_transfer_N;

  // --- 4. Estimate Vertical Force per Tire ---
  // Lateral load transfer. Positive ay_mps2 (accel to vehicle's right) shifts load from Left to
  // Right tires. Caller must ensure track_width_front/rear are not zero.
  const double lateral_transfer_front_N =
    vehicle.mass.total * ay_mps2 * vehicle.dimension.cog_height /
    vehicle.dimension.track_width_front;
  const double lateral_transfer_rear_N =
    vehicle.mass.total * ay_mps2 * vehicle.dimension.cog_height /
    vehicle.dimension.track_width_rear;

  // Distribute axle load to left and right tires
  fz_tire_N.front_left = fz_f_N / 2.0 - lateral_transfer_front_N / 2.0;
  fz_tire_N.front_right = fz_f_N / 2.0 + lateral_transfer_front_N / 2.0;
  fz_tire_N.rear_left = fz_r_N / 2.0 - lateral_transfer_rear_N / 2.0;
  fz_tire_N.rear_right = fz_r_N / 2.0 + lateral_transfer_rear_N / 2.0;

  return fz_tire_N;
}
}  // namespace tam::helpers::vehicle_dynamics
