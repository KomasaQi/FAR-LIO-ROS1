// Copyright 2023 Simon Hoffmann
#pragma once
#include <iostream>
#include <numeric>
#include <stdexcept>
#include <vector>
namespace tam::helpers
{
template <typename E>
constexpr typename std::underlying_type<E>::type to_underlying(E e) noexcept
{
  return static_cast<typename std::underlying_type<E>::type>(e);
}
template <typename T>
int sgn(T val)
{
  return (T(0) < val) - (val < T(0));
}
template <typename T>
std::vector<T> linspace(T start, T end, size_t num)
{
  std::vector<T> linspaced_array{};
  if (num <= 0) {
    return linspaced_array;
  }
  if (num == 1) {
    linspaced_array.push_back(start);
    return linspaced_array;
  }
  T delta = (end - start) / (num - 1);
  for (size_t i = 0; i < num; ++i) {
    linspaced_array.push_back(start + delta * i);
  }
  // Ensure the last point is exactly 'end' to avoid floating point issues if num > 1
  if (num > 1) {
    linspaced_array.back() = end;
  }
  return linspaced_array;
}
template <typename T>
void normalize_vector(std::vector<T> & vec)
{
  T sum = std::accumulate(vec.begin(), vec.end(), 0.0);
  if (sum > 0.0f) {
    for (T & val : vec) {
      val /= sum;
    }
  } else {
    std::cerr << "Warning: Sum of vector elements is zero. Cannot normalize." << std::endl;
  }
}
}  // namespace tam::helpers
