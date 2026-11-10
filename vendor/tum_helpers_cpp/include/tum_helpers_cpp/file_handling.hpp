// Copyright 2023 Simon Hoffmann
#pragma once
#include <bit>      // std::bit_cast
#include <cstdint>  // uint64_t
#include <eigen3/Eigen/Dense>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
namespace tam::helpers::files
{
// inline double qnan_bits() noexcept {
//   // IEEE-754 quiet NaN: exponent = all 1s, mantissa != 0.
//   // Using a common quiet-NaN payload (0x7ff8...).
//   constexpr uint64_t QNAN = 0x7ff8000000000001ULL;
//   return std::bit_cast<double>(QNAN);
// }

// template <typename M>
// static M load_csv(
//     const std::string& path,
//     const int header_lines = 1,
//     const char comment_char = '#')
// {
//     if (!std::filesystem::exists(path)) {
//         throw std::runtime_error("[tam::helpers::files::load_csv]: File does not exist " + path);
//     }

//     std::ifstream indata(path);
//     std::string line;
//     std::vector<double> values;
//     std::size_t rows = 0;
//     int n = 1;

//     auto trim = [](std::string& s) {
//         const char* ws = " \t\r\n";
//         s.erase(0, s.find_first_not_of(ws));
//         s.erase(s.find_last_not_of(ws) + 1);
//     };

//     while (std::getline(indata, line)) {
//         if (!line.empty() && line.front() == comment_char) continue;
//         if (n++ <= header_lines) continue;

//         std::stringstream lineStream(line);
//         std::string cell;
//         while (std::getline(lineStream, cell, ',')) {
//             trim(cell);
//             if (cell.empty() ||
//                 cell == "nan" || cell == "NaN" || cell == "NAN")
//             {
//                 values.push_back(qnan_bits());
//             } else {
//                 try {
//                     values.push_back(std::stod(cell));
//                 } catch (...) {
//                     // On parse failure, store our deterministic NaN too.
//                     values.push_back(qnan_bits());
//                 }
//             }
//         }
//         ++rows;
//     }
//     return Eigen::Map<const Eigen::Matrix<
//         typename M::Scalar,
//         M::RowsAtCompileTime,
//         M::ColsAtCompileTime,
//         Eigen::RowMajor>>(values.data(), rows, values.size() / rows);
// }

inline double qnan_bits() noexcept
{
  constexpr uint64_t QNAN = 0x7ff8000000000001ULL;  // quiet NaN payload
  return std::bit_cast<double>(QNAN);
}
template <typename M>
static M load_csv(
  const std::string & path, const int header_lines = 1, const char comment_char = '#')
{
  if (!std::filesystem::exists(path)) {
    throw std::runtime_error("[tam::helpers::files::load_csv]: File does not exist " + path);
  }

  std::ifstream indata(path);
  std::string line;
  std::vector<double> values;
  std::size_t rows = 0;
  int n = 1;

  auto trim = [](std::string & s) {
    const char * ws = " \t\r\n";
    const auto first = s.find_first_not_of(ws);
    if (first == std::string::npos) {
      s.clear();
      return;
    }  // all whitespace
    const auto last = s.find_last_not_of(ws);
    s.erase(last + 1);
    s.erase(0, first);
  };

#ifndef NDEBUG
  std::vector<std::size_t> cols_per_row;
  cols_per_row.reserve(4096);
#endif

  while (std::getline(indata, line)) {
    if (!line.empty() && line.front() == comment_char) continue;
    if (n++ <= header_lines) continue;

    std::stringstream lineStream(line);
    std::string cell;
    std::size_t cols_this_row = 0;

    while (std::getline(lineStream, cell, ',')) {
      trim(cell);
      if (cell.empty() || cell == "nan" || cell == "NaN" || cell == "NAN") {
        values.push_back(qnan_bits());
      } else {
        try {
          values.push_back(std::stod(cell));  // OK under -ffast-math/finite-math
        } catch (...) {
          values.push_back(qnan_bits());  // parse failure -> NaN
        }
      }
      ++cols_this_row;
    }
    ++rows;

#ifndef NDEBUG
    cols_per_row.push_back(cols_this_row);
#endif
  }

  // --- Debug diagnostics ----------------------------------------------------
#ifndef NDEBUG
  if (rows == 0) {
    std::cerr << "[CSV DEBUG] " << path << ": no data rows after header.\n";
  } else {
    std::size_t min_cols = cols_per_row[0], max_cols = cols_per_row[0];
    for (auto c : cols_per_row) {
      min_cols = std::min(min_cols, c);
      max_cols = std::max(max_cols, c);
    }
    const std::size_t total_vals = values.size();
    const std::size_t inferred_cols = (rows ? (total_vals / rows) : 0);

    std::cerr << "[CSV DEBUG] " << path << "\n"
              << "  rows=" << rows << "  total_values=" << total_vals
              << "  inferred_cols=" << inferred_cols << "  min_cols_per_row=" << min_cols
              << "  max_cols_per_row=" << max_cols << "\n";

    // If ragged, print first few offenders
    if (min_cols != max_cols) {
      std::cerr << "  WARNING: ragged rows detected. First 10 row widths:\n    ";
      for (std::size_t r = 0; r < std::min<std::size_t>(10, cols_per_row.size()); ++r) {
        std::cerr << cols_per_row[r]
                  << (r + 1 < std::min<std::size_t>(10, cols_per_row.size()) ? ", " : "");
      }
      std::cerr << "\n";
    }

    if (rows * inferred_cols != total_vals) {
      std::cerr
        << "  WARNING: rows * inferred_cols != total_values (" << rows << " * " << inferred_cols
        << " != " << total_vals << ").\n"
        << "           This means truncated or extra cells exist; mapping may drop tail values.\n";
    }
  }
#endif
  // --------------------------------------------------------------------------

  return Eigen::Map<const Eigen::Matrix<
    typename M::Scalar, M::RowsAtCompileTime, M::ColsAtCompileTime, Eigen::RowMajor>>(
    values.data(), rows, rows ? (values.size() / rows) : 0);
}
// template <typename M>
// static M load_csv(
//   const std::string & path, const int header_lines = 1, const char comment_char = '#')
// {
//   if (!std::filesystem::exists(path)) {
//     throw std::runtime_error("[tam::helpers::files::load_csv]: File does not exist " + path);
//   }
//   std::ifstream indata;
//   indata.open(path);
//   std::string line;
//   std::vector<double> values;
//   uint rows = 0;
//   uint n = 1;
//   while (std::getline(indata, line)) {
//     if (line.at(0) == comment_char) continue;
//     if (n <= header_lines) {
//       ++n;
//       continue;
//     }
//     std::stringstream lineStream(line);
//     std::string cell;
//     while (std::getline(lineStream, cell, ',')) {
//       if (cell.empty()){
//         values.push_back(std::nan("1"));
//       } else {
//         values.push_back(std::stod(cell));
//       }
//     }
//     ++n;
//     ++rows;
//   }
//   return Eigen::Map<const Eigen::Matrix<
//     typename M::Scalar, M::RowsAtCompileTime, M::ColsAtCompileTime, Eigen::RowMajor>>(
//     values.data(), rows, values.size() / rows);
// }

static std::vector<std::string> get_csv_header(
  const std::string & path, const int header_lines = 1, const char comment_char = '#')
{
  if (!std::filesystem::exists(path)) {
    throw std::runtime_error("[tam::helpers::files::load_csv]: File does not exist " + path);
  }
  std::ifstream indata;
  indata.open(path);
  std::string line;
  std::vector<std::string> values;
  uint rows = 0;
  uint n = 1;
  while (std::getline(indata, line)) {
    if (line.at(0) == comment_char) continue;
    if (n == header_lines) {
      std::stringstream lineStream(line);
      std::string cell;
      while (std::getline(lineStream, cell, ',')) {
        values.push_back(cell);
      }
      return values;
    }
    ++n;
    ++rows;
  }
}
// Function to get comments from csv file (starting with #) and extract lines that contain a keyword
static std::vector<std::vector<std::string>> get_csv_comments(
  const std::string & path, std::string keyword)
{
  char comment_char = '#';
  if (!std::filesystem::exists(path)) {
    throw std::runtime_error("[tam::helpers::files::load_csv]: File does not exist " + path);
  }

  std::ifstream indata;
  indata.open(path);
  std::string line;
  std::vector<std::vector<std::string>> values;

  while (std::getline(indata, line)) {
    if (line.at(0) == comment_char) {
      size_t keyword_position = line.find(keyword);
      if (keyword_position != std::string::npos) {  // neuer Block wenn keyword gefunden
        std::vector<std::string> block;
        size_t base_indent =
          line.find_first_not_of(" ", 1);  // Position von erstem Leerzeichen nach dem Kommentar
        block.push_back(line);

        while (std::getline(indata, line)) {
          if (line.at(0) != comment_char) {
            break;
          }
          size_t indent = line.find_first_not_of(" ", 1);
          if (indent > base_indent) {
            block.push_back(line);
          } else {
            break;
          }
        }
        values.push_back(block);
      }
    }
  }
  return values;
}
}  // namespace tam::helpers::files
   // namespace tam::helpers::files
