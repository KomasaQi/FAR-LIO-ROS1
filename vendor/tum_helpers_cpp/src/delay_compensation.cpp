// Copyright 2023 Marcel Weinmann
#include "tum_helpers_cpp/delay_compensation.hpp"
tam::helpers::DelayCompensation::DelayCompensation(int buffer_size, double step_size)
{
  if (step_size > 1e-06) {
    step_size_ = step_size;
    init_cycle_length_ = buffer_size;
  } else {
    throw std::invalid_argument("[DelayCompensation]: step_size has to be larger than 1e-06");
  }
  odometry_fifo_.resize(buffer_size);
}
/**
 * @brief Update the internal deque containing the last N state estimation outputs
 *
 * @param[in] input         - tam::types::control::Odometry:
 *                            current state estimation output
 */
void tam::helpers::DelayCompensation::update_vehicle_odometry(
  const tam::types::control::Odometry & se_odometry)
{
  if (init_cycle_length_ != 0) {
    init_cycle_length_--;
  }

  // update the state estimation FiFo buffer
  odometry_fifo_.push_front(se_odometry);
  odometry_fifo_.pop_back();
}
/**
 * @brief Compensate the delay of the input odometry
 *
 * @param[in] input         - tam::types::control::Odometry:
 *                            current state estimation output
 * @param[in] delta_t_s     - double:
 *                            time delay to compensate in seconds
 */
bool tam::helpers::DelayCompensation::compensate(
  tam::types::control::Odometry & input_odometry, double delta_t_s)
{
  int step_delay = static_cast<int>(delta_t_s / step_size_);
  double fractional_step_delay = delta_t_s / step_size_ - step_delay;

  // catch negative delta_t_s
  if (step_delay < 0 || fractional_step_delay < 0.0 || init_cycle_length_ != 0) {
    return true;
  }

  if (odometry_fifo_.size() > static_cast<size_t>(step_delay + 1)) {
    // get all relevant odometries and create a lambda to interpolate fractional step delays
    auto interpolate = [&](auto current, auto next) {
      return (current - next) * fractional_step_delay;
    };
    const auto & last_odometry = odometry_fifo_[0];
    const auto & delayed_odometry = odometry_fifo_[step_delay];
    const auto & next_delayed_odometry = odometry_fifo_[step_delay + 1];

    // compensate delay in position
    input_odometry.position_m +=
      last_odometry.position_m - delayed_odometry.position_m +
      interpolate(delayed_odometry.position_m, next_delayed_odometry.position_m);

    // compensate delay in orientation
    auto interpolate_angle = [&](auto current, auto next) {
      return tam::helpers::geometry::normalize_angle(current - next) * fractional_step_delay;
    };
    input_odometry.orientation_rad = tam::helpers::geometry::normalize_angle(
      input_odometry.orientation_rad + last_odometry.orientation_rad -
      delayed_odometry.orientation_rad +
      interpolate_angle(delayed_odometry.orientation_rad, next_delayed_odometry.orientation_rad));

    // compensate delay in linear velocity
    input_odometry.velocity_mps +=
      last_odometry.velocity_mps - delayed_odometry.velocity_mps +
      interpolate(delayed_odometry.velocity_mps, next_delayed_odometry.velocity_mps);

    // compensate delay in angular velocity
    input_odometry.angular_velocity_radps +=
      last_odometry.angular_velocity_radps - delayed_odometry.angular_velocity_radps +
      interpolate(
        delayed_odometry.angular_velocity_radps, next_delayed_odometry.angular_velocity_radps);

    // delay was successfully compensated
    return true;
  } else {
    return false;
  }
}
