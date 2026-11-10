// Copyright 2025 Maximilian Leitenstern
// C++ Implementation of CCMA (Curvature-Weighted Moving Average)
// https://github.com/UniBwTAS/ccma

#include <tum_helpers_cpp/ccma.hpp>
namespace tam::helpers
{
// Constructor for the CCMA class
CCMA::CCMA(
  const size_t w_ma, const size_t w_cc, const std::string & distrib, const std::string distrib_ma,
  const std::string distrib_cc, const float rho_ma, const float rho_cc)
: w_ma_(w_ma), w_cc_(w_cc), distrib_(distrib), rho_ma_(rho_ma), rho_cc_(rho_cc)
{
  // Initialize the CCMA weights
  this->w_ccma_ = w_ma + w_cc + 1;
  this->distrib_ma_ = distrib_ma.empty() ? distrib : distrib_ma;
  this->distrib_cc_ = distrib_cc.empty() ? distrib : distrib_cc;
  // Compute the weights
  this->weights_ma_ = this->get_weights(w_ma_, this->distrib_ma_, rho_ma_);
  this->weights_cc_ = this->get_weights(w_cc_, this->distrib_cc_, rho_cc_);
  std::cout << "Initialized CCMA with parameters:" << std::endl;
  std::cout << "w_ma: " << w_ma_ << ", w_cc: " << w_cc_ << ", distrib: " << distrib
            << ", rho_ma: " << rho_ma_ << ", rho_cc: " << rho_cc_ << std::endl;
}
// ================================================================================================
std::vector<Eigen::Vector3f> CCMA::filter(
  const std::vector<Eigen::Vector3f> & points, const std::string & mode, const bool cc_mode)
{
  // Check if the mode is valid
  if (std::find(modes_.begin(), modes_.end(), mode) == modes_.end()) {
    throw std::invalid_argument("Invalid mode");
  }
  // Check if enough points are given
  if (points.size() < 3 || points.size() < this->w_ccma_ * 2 + 1) {
    std::cerr << "Not enough points for CCMA. Minimum required: " << this->w_ccma_ * 2 + 1
              << std::endl;
    return points;
  }
  // Allocate empty vector for points
  std::vector<Eigen::Vector3f> points_{};
  if (mode == "padding") {
    size_t n_padding = cc_mode ? this->w_ccma_ : this->w_ma_;
    // Reserve space to avoid multiple reallocations
    points_.reserve(points.size() + 2 * n_padding);
    // Add n_padding copies of the first element.
    const Eigen::Vector3f & first_val = points.front();
    for (size_t i = 0; i < n_padding; ++i) {
      points_.push_back(first_val);
    }
    // Add all original elements from 'points'.
    points_.insert(points_.end(), points.begin(), points.end());

    // Add n_padding copies of the last element.
    const Eigen::Vector3f & last_val = points.back();
    for (size_t i = 0; i < n_padding; ++i) {
      points_.push_back(last_val);
    }
  }

  if (mode == "wrapping") {
    size_t n_padding = cc_mode ? this->w_ccma_ : this->w_ma_;
    // Reserve space to avoid multiple reallocations
    points_.reserve(points.size() + 2 * n_padding);

    // Add the last n_padding elements from 'points' to the beginning of 'points_'.
    if (n_padding > 0 && n_padding <= points.size()) {
      points_.insert(points_.end(), points.end() - n_padding, points.end());
    }
    // Add all original elements from 'points'.
    points_.insert(points_.end(), points.begin(), points.end());

    // Add the first n_padding elements from 'points' to the end of 'points_'.
    if (n_padding > 0 && n_padding <= points.size()) {
      points_.insert(points_.end(), points.begin(), points.begin() + n_padding);
    }
  }
  return this->filter_impl(points_, this->w_ma_, this->w_cc_, cc_mode);
}
// ================================================================================================
std::vector<Eigen::Vector3f> CCMA::filter_impl(
  const std::vector<Eigen::Vector3f> & points, const size_t w_ma, const size_t w_cc,
  const bool cc_mode)
{
  size_t w_ccma = w_ma + w_cc + 1;
  // Calculate moving-average points
  std::vector<Eigen::Vector3f> points_ma = this->get_ma_points(points, this->weights_ma_[w_ma]);
  if (!cc_mode) return points_ma;

  //  Calculate curvature vectors & curvatures
  std::vector<Eigen::Vector3f> curvature_vectors = this->get_curvature_vectors(points_ma);

  std::vector<float> curvatures(points_ma.size());  // Initialize with zeros
  for (size_t i = 0; i < curvature_vectors.size(); ++i) {
    curvatures[i] = curvature_vectors[i].norm();
  }

  // Calculate alphas (angles defined two consecutive points defined by the assumption of const.
  // curvature)
  std::vector<float> alphas = this->get_alphas(points_ma, curvatures);

  // Calculate radii
  std::vector<float> radii_ma =
    this->get_normalized_ma_radii(alphas, w_ma, this->weights_ma_[w_ma]);

  // Allocate
  std::vector<Eigen::Vector3f> points_ccma(points.size() - 2 * w_ccma);

  for (size_t idx = 0; idx < points.size() - 2 * w_ccma; ++idx) {
    // Get tangent vector for the calculation of the shifting
    Eigen::Vector3f unit_tangent =
      (points_ma[w_cc + idx + 1 + 1] - points_ma[w_cc + idx - 1 + 1]).normalized();

    // Calculate the weighted shift
    Eigen::Vector3f shift = Eigen::Vector3f::Zero();
    for (size_t idx_cc = 0; idx_cc < 2 * w_cc + 1; ++idx_cc) {
      // In case the path is straight, no curvature correction is necessary
      if (std::abs(curvatures[idx + w_cc + 1]) < 1.0e-6) {
        continue;
      }

      Eigen::Vector3f u_vec = (curvature_vectors[idx + w_cc + 1]).normalized();
      float weight = this->weights_cc_[w_cc][idx_cc];
      float shift_magnitude =
        (1 / curvatures[idx + idx_cc + 1]) * (1 / radii_ma[idx + idx_cc + 1] - 1);
      shift += u_vec * weight * shift_magnitude;
    }

    // Reconstruction
    points_ccma[idx] = points_ma[idx + w_cc + 1] + unit_tangent.cross(shift);
  }

  return points_ccma;
}
// ================================================================================================
std::vector<std::vector<float>> CCMA::get_weights(size_t w, const std::string & distrib, float rho)
{
  std::vector<std::vector<float>> weight_list;
  weight_list.reserve(w + 1);

  if (distrib == "normal") {
    boost::math::normal_distribution<> standard_normal_dist(0.0, 1.0);
    float x_start = boost::math::quantile(standard_normal_dist, (1.0 - rho) / 2.0);
    float x_end = boost::math::quantile(standard_normal_dist, 1.0 - ((1.0 - rho) / 2.0));

    for (size_t w_i = 0; w_i <= static_cast<size_t>(w); ++w_i) {
      size_t kernel_len = 2 * w_i + 1;
      size_t num_linspace_points = kernel_len + 1;

      std::vector<float> x_values = linspace(x_start, x_end, num_linspace_points);
      std::vector<float> current_weights(kernel_len);

      if (kernel_len == 0) {  // Should not happen if w_i >= 0
        weight_list.push_back({});
        continue;
      }

      for (size_t idx = 0; idx < kernel_len; ++idx) {
        if (idx + 1 < x_values.size()) {
          current_weights[idx] = boost::math::cdf(standard_normal_dist, x_values[idx + 1]) -
                                 boost::math::cdf(standard_normal_dist, x_values[idx]);
        } else {
          // Should ideally not be reached if logic is correct, implies an issue with x_values
          // sizing
          current_weights[idx] = 0.0;
        }
      }

      if (rho != 0.0) {
        for (float & val : current_weights) {
          val /= rho;
        }
      }
      weight_list.push_back(current_weights);
    }
  } else if (distrib == "uniform") {
    for (size_t w_i = 0; w_i <= w; ++w_i) {
      size_t kernel_len = 2 * w_i + 1;
      std::vector<float> current_weights(kernel_len, 1.0 / static_cast<float>(kernel_len));
      weight_list.push_back(current_weights);
    }
  } else if (distrib == "pascal") {
    for (size_t w_i = 0; w_i <= w; ++w_i) {
      size_t pascal_row_idx = w_i * 2;
      std::vector<float> row = get_pascal_row(pascal_row_idx);
      normalize_vector(row);
      weight_list.push_back(row);
    }
  } else if (distrib == "hanning") {
    for (size_t w_i = 0; w_i <= w; ++w_i) {
      size_t final_kernel_length =
        2 * w_i + 1;  // This is `window_size` in Python's get_hanning_kernel call

      if (final_kernel_length <= 0) {
        weight_list.push_back({});  // Or throw error
        continue;
      }

      std::vector<float> kernel(final_kernel_length);
      if (final_kernel_length == 1) {  // Special case for single point kernel
        kernel[0] = 1.0;
      } else {
        // Python: window_size += 2; ... [1:-1]
        // Effective N for cosine argument generation is final_kernel_length + 2
        size_t hanning_N_param = final_kernel_length + 2;
        for (size_t i = 0; i < final_kernel_length; ++i) {
          // Python's arange index `k` for the sliced part goes from 1 to hanning_N_param-2
          // current `i` corresponds to `k-1`. So `k = i + 1`.
          float k_py = static_cast<float>(i + 1);
          kernel[i] =
            0.5 * (1.0 - std::cos(2.0 * M_PI * k_py / (static_cast<float>(hanning_N_param) - 1.0)));
        }
        normalize_vector(kernel);
      }
      weight_list.push_back(kernel);
    }
  } else {
    throw std::invalid_argument(
      "Distribution must either be 'uniform', 'pascal', 'hanning', or 'normal'. Received: " +
      distrib);
  }
  return weight_list;
}
// ================================================================================================
std::vector<Eigen::Vector3f> CCMA::get_ma_points(
  const std::vector<Eigen::Vector3f> & points, const std::vector<float> & weights)
{
  if (weights.empty()) {
    // Behavior of np.convolve(data, [], 'valid') is to return data.
    return points;
  }
  if (points.empty()) {
    return {};
  }

  size_t num_points = points.size();
  size_t num_weights = weights.size();

  size_t output_length = std::max(num_points, num_weights) - std::min(num_points, num_weights) + 1;

  // Initialize the result vector
  std::vector<Eigen::Vector3f> convolved_points(output_length);

  // Perform 1D convolution for each dimension (x, y, z).
  for (size_t i = 0; i < output_length; ++i) {
    float point_conv_x = 0.0f;
    float point_conv_y = 0.0f;
    float point_conv_z = 0.0f;

    for (size_t j = 0; j < num_weights; ++j) {
      // points[i + j] is the (i+j)-th data point in the input sequence.
      // weights[j] is the j-th weight in the convolution kernel.
      point_conv_x += points[i + j].x() * weights[j];
      point_conv_y += points[i + j].y() * weights[j];
      point_conv_z += points[i + j].z() * weights[j];
    }
    convolved_points[i] = Eigen::Vector3f(point_conv_x, point_conv_y, point_conv_z);
  }
  return convolved_points;
}
// ================================================================================================
std::pair<std::vector<size_t>, std::vector<size_t>> CCMA::get_descending_width()
{
  size_t w_ma_cur = this->w_ma_;
  size_t w_cc_cur = this->w_cc_;
  std::pair<std::vector<size_t>, std::vector<size_t>> descending_width_list{};

  while (w_ma_cur > 0 || w_cc_cur > 0) {
    if (w_cc_cur >= w_ma_cur) {
      w_cc_cur -= 1;
    } else {
      w_ma_cur -= 1;
    }
    descending_width_list.first.push_back(w_ma_cur);
    descending_width_list.second.push_back(w_cc_cur);
  }
  return descending_width_list;
}
// ================================================================================================
std::vector<Eigen::Vector3f> CCMA::get_curvature_vectors(
  const std::vector<Eigen::Vector3f> & points)
{
  std::vector<Eigen::Vector3f> curvature_vectors(points.size());
  for (size_t i = 1; i < points.size() - 1; ++i) {
    Eigen::Vector3f p0 = points[i - 1];
    Eigen::Vector3f p1 = points[i];
    Eigen::Vector3f p2 = points[i + 1];
    Eigen::Vector3f v1 = p1 - p0;
    Eigen::Vector3f v2 = p2 - p1;
    Eigen::Vector3f cross_product = v1.cross(v2);
    float cross_norm = cross_product.norm();
    float curvature = 0.0f;
    if (cross_norm > 0.0f) {
      float radius = (v1.norm() * v2.norm() * (p2 - p0).norm()) / (2.0f * cross_norm);
      curvature = 1.0f / radius;
    }
    curvature_vectors[i] = curvature * cross_product.normalized();
  }
  return curvature_vectors;
}
// ================================================================================================
std::vector<float> CCMA::get_alphas(
  const std::vector<Eigen::Vector3f> & points, const std::vector<float> & curvatures)
{
  std::vector<float> alphas(points.size());
  for (size_t i = 1; i < points.size() - 1; ++i) {
    float curvature = curvatures[i];
    if (std::abs(curvature) > 0.0f) {
      float radius = 1.0f / curvature;
      float dist_neigbors = (points[i + 1] - points[i - 1]).norm();
      alphas[i] = std::asin((dist_neigbors / 2.0f) / radius);
    } else {
      alphas[i] = 0.0f;
    }
  }
  return alphas;
}
// ================================================================================================
std::vector<float> CCMA::get_normalized_ma_radii(
  const std::vector<float> & alphas, const size_t w_ma, const std::vector<float> & weights)
{
  std::vector<float> radii_ma(alphas.size());
  for (size_t i = 1; i < alphas.size() - 1; ++i) {
    float radius = 1.0f * weights[w_ma];
    for (size_t k = 1; k <= w_ma; ++k) {
      radius += 2.0f * std::cos(alphas[i] * k) * weights[w_ma + k];
    }
    radii_ma[i] = std::max(0.35f, radius);
  }
  return radii_ma;
}
// ================================================================================================
std::vector<float> CCMA::get_pascal_row(size_t row_index)
{
  std::vector<float> current_row{};
  current_row.push_back(1.0);

  if (row_index == 0) {
    return current_row;
  }

  std::vector<float> prev_row = get_pascal_row(row_index - 1);

  for (size_t i = 1; i < prev_row.size(); ++i) {
    current_row.push_back(prev_row[i - 1] + prev_row[i]);
  }
  current_row.push_back(1.0);  // Append the final 1.0 for rows > 0

  return current_row;
}
}  // namespace tam::helpers
