// Copyright 2025 Maximilian Leitenstern
// C++ Implementation of CCMA (Curvature-Weighted Moving Average)
// https://github.com/UniBwTAS/ccma
#pragma once

#include <eigen3/Eigen/Geometry>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

#include "common.hpp"
// Boost includes for normal distribution
#include <boost/math/constants/constants.hpp>  // For pi
#include <boost/math/distributions/normal.hpp>
namespace tam::helpers
{
/**
 * @brief CCMA (Curvature-Weighted Moving Average) class for filtering 3D points.
 *
 * This class implements the CCMA algorithm presented in the paper:
 * https://www.researchgate.net/publication/372692752_A_Simple_and_Model-Free_Path_Filtering_Algorithm_for_Smoothing_and_Accuracy
 * Custom implementation as the original code is only in python.
 */
class CCMA
{
public:
  /**
   * @brief Constructor for the CCMA class.
   *
   * @param w_ma Width of the moving average window.
   * @param w_cc Width of the curvature window.
   * @param distrib Kernel type used for filtering:
   *   - "normal": Truncated normal distribution with truncation area rho_ma + rho_cc
   *   - "uniform": Uniform distribution of weights
   *   - "pascal": Kernel based on rows of Pascal's triangle, a discretized
          version of the normal distribution. (Default is "pascal")
       - "hanning": A common kernel in signal processing, less accurate but
          offers good smoothing characteristics.
   * @param distrib_ma Kernel type for the moving average distribution (defaults to distrib).
   * @param distrib_cc Kernel type for the curvature distribution (defaults to distrib).
   * @param rho_ma Truncation area for the normal distribution in the moving average.
   * @param rho_cc Truncation area for the normal distribution curvature correction.
   *
   * @note The `distrib` parameter specifies the type of kernel used for filtering;
     the kernel is the shape of weights used in the convolution process.
   * @note `rho_ma` and `rho_cc` apply only when the "normal" kernel is selected and
     control the truncation of the normal distribution.
   * @note If `rho_ma` or `rho_cc` approximates 0, the "normal" kernel approximates
     the "uniform" kernel.
   */
  CCMA(
    const size_t w_ma = 5, const size_t w_cc = 3, const std::string & distrib = "pascal",
    const std::string distrib_ma = "", const std::string distrib_cc = "", const float rho_ma = 0.95,
    const float rho_cc = 0.95);
  /**
   * @brief Filters a vector of 3D points using the CCMA algorithm.
   *
   * @param points Vector of 3D points to be filtered.
   * @param mode Mode for filtering (default is "padding"):
   *   - "none": No preprocessing, not preserving length.
   *   - "padding": Pads the input with copies of the first and last elements. Preserves length.
   *   - "wrapping": Treats the input as cyclic, wrapping around.
          to the beginning and the first n_padding elements to the end.
   * @param cc_mode If true, applies curvature correction. If false, only moving average is applied.
   *
   * @return Filtered vector of 3D points.
   */
  std::vector<Eigen::Vector3f> filter(
    const std::vector<Eigen::Vector3f> & points, const std::string & mode = "padding",
    const bool cc_mode = true);

private:
  // Member variables
  size_t w_ma_;             // Width of the moving average window
  size_t w_cc_;             // Width of the curvature window
  size_t w_ccma_;           // Combined width of the moving average and curvature window
  std::string distrib_;     // Distribution type for the kernel
  std::string distrib_ma_;  // Distribution type for the moving average kernel
  std::string distrib_cc_;  // Distribution type for the curvature kernel
  float rho_ma_;            // Truncation area for the normal distribution in the moving average
  float rho_cc_;  // Truncation area for the normal distribution in the curvature correction
  std::vector<std::vector<float>> weights_ma_;  // Weights for the moving average kernel
  std::vector<std::vector<float>> weights_cc_;  // Weights for the curvature kernel
  std::vector<std::string> modes_{"none", "padding", "wrapping"};  // Available modes for filtering

private:
  /**
   * @brief Internal function to perform the filtering operation.
   *
   * @param points Vector of 3D points to be filtered.
   * @param w_ma Width of the moving average window.
   * @param w_cc Width of the curvature window.
   * @param cc_mode If true, applies curvature correction. If false, only moving average is applied.
   *
   * @return Filtered vector of 3D points.
   */
  std::vector<Eigen::Vector3f> filter_impl(
    const std::vector<Eigen::Vector3f> & points, const size_t w_ma, const size_t w_cc,
    const bool cc_mode = true);
  /**
   * @brief Generates a list of weight arrays (kernels) based on the specified distribution.
   *
   * @param w The base width for the kernels. Generates w+1 kernels.
   * @param distrib The type of distribution: "normal", "uniform", "pascal", "hanning".
   * @param rho Truncation area for "normal" distribution (e.g., 0.95 for 95%).
   * @return A list of 1D float vectors, each representing kernel weights.
   * @throws std::invalid_argument if distrib is not recognized.
   */
  std::vector<std::vector<float>> get_weights(size_t w, const std::string & distrib, float rho);
  /**
   * @brief Apply convolution to each dimension of the points and combine the results.
   *
   * @note Performs valid convolution, meaning the output size is
   *       max(points.size(), weights.size()) - min(points.size(), weights.size()) + 1.
   * @param points Vector of 3D points to be convolved.
   * @param weights Vector of weights for the convolution.
   * @return A vector of 3D points after applying the convolution.
   */
  std::vector<Eigen::Vector3f> get_ma_points(
    const std::vector<Eigen::Vector3f> & points, const std::vector<float> & weights);
  /**
   * @brief Generates a sequence of width parameters with descending values.
   *
   * @brief reduces the width parameters of the members variables w_ma_ and w_cc_
   * iteratively ensuring that at each step the larger is reduced. The process
   * continues until both w_ma_ and w_cc_ are reduced to 0.
   *
   * @return A pair of vectors of floats for w_ma(first) and w_cc(second)
   * @throws std::invalid_argument if distrib is not recognized.
   */
  std::pair<std::vector<size_t>, std::vector<size_t>> get_descending_width();
  /**
   * @brief Calculate curvature vectors for a given set of points.
   *
   * @param points Vector of 3D points.
   * @return A vector of curvature vectors for each point. Elements at the
   *         beginning and end of the vector are set to zero.
   */
  std::vector<Eigen::Vector3f> get_curvature_vectors(const std::vector<Eigen::Vector3f> & points);
  /**
   * @brief Calculate angles (alphas) between consecutive points assuming three points
   *        form a circle.
   *
   * @param points Vector of 3D points.
   * @param curvatures Vector of curvature values for each point.
   * @return A vector of angles (alphas) for each point. Elements at the
   *         beginning and end of the vector are set to zero.
   */
  std::vector<float> get_alphas(
    const std::vector<Eigen::Vector3f> & points, const std::vector<float> & curvatures);
  /**
   * @brief Calculate normalized radii for each points based on moving average
   *        and curvature values.
   *
   * @param alphas Vector of angles (alphas) for each point.
   * @param w_ma Width of the moving average window.
   * @param weights Vector of weights for the moving average.
   * @return A vector of normalized radii for each point. Radii are adjusted with
   *         minimum threshold to avoid unstable corrections.
   */
  std::vector<float> get_normalized_ma_radii(
    const std::vector<float> & alphas, const size_t w_ma, const std::vector<float> & weights);

private:
  // Helper functions
  /**
   * @brief Generates a row from Pascal's triangle.
   */
  std::vector<float> get_pascal_row(size_t row_index);
};
}  // namespace tam::helpers
