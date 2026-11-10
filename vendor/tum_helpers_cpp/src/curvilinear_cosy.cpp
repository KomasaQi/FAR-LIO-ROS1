// Copyright 2023 Simon Hoffmann
//
// Algorithm according to: https://ieeexplore.ieee.org/abstract/document/6856487
// Implementation according to:
// https://gitlab.lrz.de/tum-cps/commonroad-drivability-checker/-/tree/master/cpp/geometry
//
#include "tum_helpers_cpp/coordinate_system/curvilinear_cosy.hpp"

#include <algorithm>
#include <optional>
#include <tuple>
#include <vector>
namespace tam::helpers::cosy
{
void CurvilinearCosy::create_segments(
  const std::vector<double> & tx, const std::vector<double> & ty, const std::vector<double> & tz)
{
  for (int i = 0; i < static_cast<int>(x_.size() - 1); ++i) {  // n_points-1 = n_segments
    this->create_segment(
      Eigen::Vector3d(x_.at(i), y_.at(i), z_.at(i)),
      Eigen::Vector3d(x_.at(i + 1), y_.at(i + 1), z_.at(i + 1)),
      Eigen::Vector3d(tx.at(i), ty.at(i), tz.at(i)),
      Eigen::Vector3d(tx.at(i + 1), ty.at(i + 1), tz.at(i + 1)));
  }
}
void CurvilinearCosy::create_segment(
  const Eigen::Vector3d & pt_1, const Eigen::Vector3d & pt_2, const Eigen::Vector3d & t_1,
  const Eigen::Vector3d & t_2)
{
  this->segment_list_.emplace_back(
    pt_1.topRows(2), pt_2.topRows(2), t_1.topRows(2), t_2.topRows(2));
}
// --------------------------------------------------------------------
// Function Overloads and Cosy API
// --------------------------------------------------------------------

Eigen::Vector2d CurvilinearCosy::convert_to_sn_global_2d(
  const double x, const double y, const double z, const double z_range) const
{
  return std::get<0>(convert_to_sn_and_get_idx_global(x, y, z, 0, z_range)).topRows(2);
}
Eigen::Vector3d CurvilinearCosy::convert_to_sn_global(
  const double x, const double y, const double z, const double yaw, const double z_range) const
{
  return std::get<0>(convert_to_sn_and_get_idx_global(x, y, z, yaw, z_range));
}
Eigen::Vector3d CurvilinearCosy::convert_to_sn_global(
  const tam::types::control::Odometry & odom, const double z_range) const
{
  return std::get<0>(convert_to_sn_and_get_idx_global(
    odom.position_m.x, odom.position_m.y, odom.position_m.z, odom.orientation_rad.z, z_range));
}
std::tuple<Eigen::Vector3d, float> CurvilinearCosy::convert_to_sn_and_get_idx_global(
  const tam::types::control::Odometry & odom, const double z_range) const
{
  return convert_to_sn_and_get_idx_global(
    odom.position_m.x, odom.position_m.y, odom.position_m.z, odom.orientation_rad.z, z_range);
}
std::tuple<Eigen::Vector2d, float> CurvilinearCosy::convert_to_sn_and_get_idx_global_2d(
  const double x, const double y, const double z, const double z_range) const
{
  auto tmp = convert_to_sn_and_get_idx_global(x, y, z, 0, z_range);
  return {std::get<0>(tmp).topRows(2), std::get<1>(tmp)};
}
Eigen::Vector2d CurvilinearCosy::convert_to_sn_window_2d(
  const double x, const double y, const double z, const double s_start, const double s_end,
  const double z_range) const
{
  return std::get<0>(convert_to_sn_and_get_idx_window(x, y, z, 0, s_start, s_end, z_range))
    .topRows(2);
}
Eigen::Vector3d CurvilinearCosy::convert_to_sn_window(
  const double x, const double y, const double z, const double yaw, const double s_start,
  const double s_end, const double z_range) const
{
  return std::get<0>(convert_to_sn_and_get_idx_window(x, y, z, yaw, s_start, s_end, z_range));
}
Eigen::Vector3d CurvilinearCosy::convert_to_sn_window(
  const tam::types::control::Odometry & odom, const double s_start, const double s_end,
  const double z_range) const
{
  return std::get<0>(convert_to_sn_and_get_idx_window(
    odom.position_m.x, odom.position_m.y, odom.position_m.z, odom.orientation_rad.z, s_start, s_end,
    z_range));
}
std::tuple<Eigen::Vector3d, float> CurvilinearCosy::convert_to_sn_and_get_idx_window(
  const tam::types::control::Odometry & odom, const double s_start, const double s_end,
  const double z_range) const
{
  return convert_to_sn_and_get_idx_window(
    odom.position_m.x, odom.position_m.y, odom.position_m.z, odom.orientation_rad.z, s_start, s_end,
    z_range);
}
std::tuple<Eigen::Vector2d, float> CurvilinearCosy::convert_to_sn_and_get_idx_window_2d(
  const double x, const double y, const double z, const double s_start, const double s_end,
  const double z_range) const
{
  auto tmp = convert_to_sn_and_get_idx_window(x, y, z, 0, s_start, s_end, z_range);
  return {std::get<0>(tmp).topRows(2), std::get<1>(tmp)};
}
// --------------------------------------------------------------------
// Deprecated Functions
// --------------------------------------------------------------------

Eigen::Vector2d CurvilinearCosy::convert_to_sn(const double x, const double y) const
{
  return std::get<0>(convert_to_sn_and_get_idx(x, y, 0)).topRows(2);
}
Eigen::Vector3d CurvilinearCosy::convert_to_sn(
  const double x, const double y, const double yaw) const
{
  return std::get<0>(convert_to_sn_and_get_idx(x, y, yaw));
}
Eigen::Vector3d CurvilinearCosy::convert_to_sn(const tam::types::control::Odometry & odom) const
{
  return std::get<0>(
    convert_to_sn_and_get_idx(odom.position_m.x, odom.position_m.y, odom.orientation_rad.z));
}
std::tuple<Eigen::Vector3d, float> CurvilinearCosy::convert_to_sn_and_get_idx(
  const tam::types::control::Odometry & odom) const
{
  return convert_to_sn_and_get_idx(odom.position_m.x, odom.position_m.y, odom.orientation_rad.z);
}
std::tuple<Eigen::Vector2d, float> CurvilinearCosy::convert_to_sn_and_get_idx(
  const double x, const double y) const
{
  auto tmp = convert_to_sn_and_get_idx(x, y, 0);
  return {std::get<0>(tmp).topRows(2), std::get<1>(tmp)};
}
std::tuple<Eigen::Vector3d, float> CurvilinearCosy::convert_to_sn_and_get_idx(
  const double x, const double y, const double yaw) const
{
  // Convert to curvilinear coordinates globally
  // This function does not consider any window constraints
  return this->convert_to_sn_and_get_idx_base(
    x, y, 0.0, yaw, 0.0, this->length_, closed_cosy_, std::numeric_limits<double>::infinity());
}
// -------------------------------------------------------------------
// Cosy Implementations
// -------------------------------------------------------------------

std::tuple<Eigen::Vector3d, float> CurvilinearCosy::convert_to_sn_and_get_idx_global(
  const double x, const double y, const double z, const double yaw, const double z_range) const
{
  // Convert to curvilinear coordinates globally
  // This function does not consider any window constraints
  return this->convert_to_sn_and_get_idx_base(
    x, y, z, yaw, 0.0, this->length_, closed_cosy_, z_range);
}
std::tuple<Eigen::Vector3d, float> CurvilinearCosy::convert_to_sn_and_get_idx_window(
  const double x, const double y, const double z, const double yaw, const double s_start,
  const double s_end, const double z_range) const
{
  // Convert to curvilinear coordinates within a specified window
  // Only search for projection within the segments that correspond to s_start and s_end
  if (std::isnan(s_start) || std::isnan(s_end)) {
    throw std::invalid_argument("<CurvilinearCosy> s_start or s_end is NaN");
  }

  // Ensure s_start and s_end are within valid range
  // const double safe_s_start = std::max(s_start, 0.0);
  // const double safe_s_end = std::min(s_end, this->length_);

  // Wrap s_start and s_end if they are out of bounds
  double safe_s_start = std::fmod(s_start + this->length_, this->length_);
  double safe_s_end = std::fmod(s_end + this->length_, this->length_);

  // Use the base implementation with window constraints
  return this->convert_to_sn_and_get_idx_base(
    x, y, z, yaw, safe_s_start, safe_s_end, false, z_range);
}
// -------------------------------------------------------------------
// Cosy Base
// -------------------------------------------------------------------

std::tuple<Eigen::Vector3d, float> CurvilinearCosy::convert_to_sn_and_get_idx_base(
  const double x, const double y, const double z, const double yaw, const double s_start,
  const double s_end, const bool closed_search, const double z_range) const
{
  float idx{0.0};
  int segment_idx{-1};
  // Circular counter, starts at 0 if range is exceeded. Or stops at bounds if not closed.
  tam::helpers::utility::IndexCounter idx_ctr(this->segment_list_.size() - 1, closed_cosy_);
  idx_ctr.set(this->get_closest_point_id_window(x, y, z, s_start, s_end, z_range));
  // Init values
  std::optional<Eigen::Vector2d> sd = std::nullopt;
  double seg_lambda{0.0};
  // Calc for segments around closest point
  for (int i = -range_to_check_; i <= range_to_check_; ++i) {
    double lambda;
    Eigen::Vector2d curvilinear_point =
      this->segment_list_[idx_ctr(i)].convertToCurvilinearCoords(x, y, lambda);
    // Continue if lambda not [0;1]
    if (0.0 - 1e-8 > lambda || lambda > 1.0 + 1e-8) {
      continue;
    }

    // Check if d is smaller than previous
    if (!sd.has_value() || std::abs(curvilinear_point[1]) < std::abs(sd.value()[1])) {
      sd = curvilinear_point;
      seg_lambda = lambda;
      segment_idx = idx_ctr(i);
    }
  }

  // if open and no Value found -> extrapolate and warn!
  if (!closed_search && !sd.has_value()) {
    // Extrapolate
    float out_of_bounds{0.0};
    if (
      tam::helpers::geometry::euclidean_distance(x, y, x_.front(), y_.front()) <
      tam::helpers::geometry::euclidean_distance(x, y, x_.back(), y_.back())) {
      // Closer to first point
      sd = extrapolate_front.convertToCurvilinearCoords(x, y, seg_lambda);
      segment_idx = 0;
      if (sd.has_value()) {
        out_of_bounds = std::abs(sd.value()[0] - this->segment_s0_.at(0));
      }

    } else {
      // Closer to last point
      sd = extrapolate_back.convertToCurvilinearCoords(x, y, seg_lambda);
      segment_idx = this->segment_list_.size() - 1;
      if (sd.has_value()) {
        out_of_bounds = std::abs(sd.value()[0] - this->length_);
      }
    }

    if (out_of_bounds > 0.2 && !convert_to_sn_extrapolation_warned_) {
      std::cout << "[CurvilinearCosy]: No projection on reference line found within 20 cm margin, "
                   "Extrapolating! Disabling this warning for the current Cosy\n";
      convert_to_sn_extrapolation_warned_ = true;
    }
  }

  if (!sd.has_value()) {
    throw std::invalid_argument(
      "<CurvilinearCoordinateSystem> Could not find a projection on closed Referenceline");
  }
  double heading_rl = tam::helpers::geometry::calc_heading(
    this->segment_list_.at(segment_idx).computePseudoTangentGlobal(seg_lambda), zero_heading_dir_);
  // calc a float index for output. E.g. matching point in the middle of 2&3 -> 2.5
  idx = segment_idx + seg_lambda;
  // return [s,d]: Use 3D segment length to recalc s with lambda
  return {
    Eigen::Vector3d(
      this->segment_s0_[segment_idx] + segment_length_[segment_idx] * seg_lambda, sd.value()[1],
      tam::helpers::geometry::normalize_angle(yaw - heading_rl)),
    idx};
}
int CurvilinearCosy::get_closest_point_id(const double x, const double y) const
{
  std::optional<double> min_dist{std::nullopt};
  double current_dist{0};
  int min_idx{-1};
  for (int i = 0; i < static_cast<int>(x_.size()); ++i) {
    current_dist = std::sqrt(std::pow(x_.at(i) - x, 2) + std::pow(y_.at(i) - y, 2));
    if (!min_dist.has_value() || current_dist < min_dist.value()) {
      min_dist = current_dist;
      min_idx = i;
    }
  }
  return min_idx;
}
int CurvilinearCosy::get_closest_point_id_window(
  const double x, const double y, const double z, const double s_start, const double s_end,
  const double z_range) const
{
  std::optional<double> min_dist{std::nullopt};
  double current_dist{0};
  int min_idx{-1};
  bool wraped_window = s_start > s_end;
  for (int i = 0; i < static_cast<int>(x_.size()); ++i) {
    int seg_idx = i < static_cast<int>(this->segment_s0_.size())
                    ? i
                    : static_cast<int>(this->segment_s0_.size()) -
                        1;  // s0 is starting point of segment last point still in last segment
    if (
      (wraped_window &&
       (s_start > this->segment_s0_.at(seg_idx) && s_end < this->segment_s0_.at(seg_idx))) ||
      (!wraped_window &&
       (s_start > this->segment_s0_.at(seg_idx) || s_end < this->segment_s0_.at(seg_idx))) ||
      (std::abs(z - z_.at(i)) > z_range)) {
      continue;  // Skip points outside the window
    }
    // current_dist = std::sqrt(std::pow(x_.at(i) - x, 2) + std::pow(y_.at(i) - y, 2));
    current_dist =
      (std::pow(x_.at(i) - x, 2) +
       std::pow(y_.at(i) - y, 2));  // Use squared distance to avoid sqrt for performance
    if (!min_dist.has_value() || current_dist < min_dist.value()) {
      min_dist = current_dist;
      min_idx = i;
    }
  }
  if (min_idx < 0) {
    throw std::runtime_error(
      "<CurvilinearCoordinateSystem> Could not find a nearest point that satisfies the window and "
      "z_filter constraints");
  }
  return min_idx;
}
Eigen::Vector3d CurvilinearCosy::convert_to_cartesian(
  const double s, const double d, const double yaw) const
{
  Segment seg;
  int index;
  float out_of_bounds{0};
  if (s < this->segment_s0_.at(0) && !this->closed_cosy_) {
    out_of_bounds = std::abs(s - this->segment_s0_.at(0));
    index = 0;
    seg = this->extrapolate_front;
  } else if (s > this->length_ && !this->closed_cosy_) {
    out_of_bounds = std::abs(s - this->length_);
    index = this->segment_list_.size() - 1;
    seg = this->extrapolate_back;
  } else {
    out_of_bounds = 0;
    auto idx = this->find_segment_index(s);
    if (!idx.has_value()) {
      throw std::invalid_argument("<CurvilinearCosy> s-coordinate outside of projection domain.");
    }
    index = idx.value();
    seg = this->segment_list_.at(index);
  }
  if (out_of_bounds > 0.2 && !convert_to_cartesian_extrapolation_warned_) {
    std::cout << "[CurvilinearCosy]: Provided s-coordinate lies more than 20cm before/after "
                 "referencline, Extrapolating! Disabling this warning for the current Cosy \n";
    convert_to_cartesian_extrapolation_warned_ = true;
  }

  double lambda = (s - this->segment_s0_[index]) / this->segment_length_[index];
  double yaw_global = yaw + tam::helpers::geometry::calc_heading(
                              seg.computePseudoTangentGlobal(lambda), zero_heading_dir_);
  Eigen::Vector2d pos = seg.convertToCartesianCoords(lambda, d);
  return Eigen::Vector3d(
    pos[0], pos[1], tam::helpers::geometry::normalize_angle(yaw_global));  // ADD yaw
}
Eigen::Vector2d CurvilinearCosy::convert_to_cartesian(const double s, const double d) const
{
  return convert_to_cartesian(s, d, 0).topRows(2);
}
std::optional<int> CurvilinearCosy::find_segment_index(double s) const
{
  std::optional<int> idx;
  for (int i = 0; i < static_cast<int>(this->segment_list_.size()); ++i) {
    double s_1 = this->segment_s0_[i];
    double s_2 = this->segment_s0_[i + 1];
    if (std::islessequal(s_1, s) && std::islessequal(s, s_2)) {
      idx = i;
      break;
    }
  }
  return idx;
}
Eigen::Vector3d CurvilinearCosy::convert_to_cartesian_3d(const double s, const double d) const
{
  Segment seg;
  int index;
  float out_of_bounds{0};
  if (s < this->segment_s0_.at(0) && !this->closed_cosy_) {
    out_of_bounds = std::abs(s - this->segment_s0_.at(0));
    index = 0;
    seg = this->extrapolate_front;
  } else if (s > this->length_ && !this->closed_cosy_) {
    out_of_bounds = std::abs(s - this->length_);
    index = this->segment_list_.size() - 1;
    seg = this->extrapolate_back;
  } else {
    out_of_bounds = 0;
    auto idx = this->find_segment_index(s);
    if (!idx.has_value()) {
      throw std::invalid_argument("<CurvilinearCosy> s-coordinate outside of projection domain.");
    }
    index = idx.value();
    seg = this->segment_list_.at(index);
  }
  if (out_of_bounds > 0.2 && !convert_to_cartesian_extrapolation_warned_) {
    std::cout << "[CurvilinearCosy]: Provided s-coordinate lies more than 20cm before/after "
                 "referencline, Extrapolating! Disabling this warning for the current Cosy \n";
    convert_to_cartesian_extrapolation_warned_ = true;
  }

  double lambda = (s - this->segment_s0_[index]) / this->segment_length_[index];
  Eigen::Vector2d pos = seg.convertToCartesianCoords(lambda, d);
  return Eigen::Vector3d(pos[0], pos[1], z_.at(index));
}
}  // namespace tam::helpers::cosy
