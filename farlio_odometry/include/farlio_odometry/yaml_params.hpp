// Licensed under the Apache License, Version 2.0.
//
// Shared YAML -> parameter helpers for the FAR-LIO ROS 1 nodes.
//
//  * flatten_yaml() turns a nested YAML file into flat "a.b.c" keys for
//    tam::pmg::init(), which is how upstream ROS 2 parameter overrides are fed
//    into the ROS-free libraries.
//  * ParamMap / ParamVal provide the small typed map used by the state
//    estimation node to parse its declarative sensor configuration.
#pragma once

#include <yaml-cpp/yaml.h>

#include <cstdint>
#include <map>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "param_management_cpp/initialization.hpp"
#include "param_management_cpp/types.hpp"

namespace farlio_ros1 {

// ---------------------------------------------------------------------------
// Flattening for tam::pmg::init()
// ---------------------------------------------------------------------------
inline void flatten_yaml(
  const YAML::Node& node, const std::string& prefix,
  std::vector<std::pair<std::string, tam::pmg::param_value_variant_t>>& out)
{
  if (node.IsMap()) {
    for (const auto& kv : node) {
      const std::string key = kv.first.as<std::string>();
      const std::string full = prefix.empty() ? key : prefix + "." + key;
      flatten_yaml(kv.second, full, out);
    }
    return;
  }
  if (node.IsSequence()) {
    if (prefix.empty()) return;
    bool all_bool = true, all_int = true, all_num = true;
    for (const auto& e : node) {
      if (!e.IsScalar()) {
        all_bool = all_int = all_num = false;
        continue;
      }
      const std::string v = e.Scalar();
      if (v != "true" && v != "false" && v != "True" && v != "False") all_bool = false;
      try {
        std::size_t pos = 0;
        std::stoll(v, &pos);
        if (pos != v.size()) all_int = false;
      } catch (...) {
        all_int = false;
      }
      try {
        std::size_t pos = 0;
        std::stod(v, &pos);
        if (pos != v.size()) all_num = false;
      } catch (...) {
        all_num = false;
      }
    }
    if (all_bool) {
      std::vector<bool> v;
      for (const auto& e : node) v.push_back(e.as<bool>());
      out.emplace_back(prefix, v);
    } else if (all_int) {
      std::vector<std::int64_t> v;
      for (const auto& e : node) v.push_back(e.as<std::int64_t>());
      out.emplace_back(prefix, v);
    } else if (all_num) {
      std::vector<double> v;
      for (const auto& e : node) v.push_back(e.as<double>());
      out.emplace_back(prefix, v);
    } else {
      std::vector<std::string> v;
      for (const auto& e : node) v.push_back(e.as<std::string>());
      out.emplace_back(prefix, v);
    }
    return;
  }
  if (!node.IsScalar() || prefix.empty()) return;

  const std::string raw = node.Scalar();
  if (raw == "true" || raw == "True" || raw == "false" || raw == "False") {
    out.emplace_back(prefix, node.as<bool>());
    return;
  }
  try {
    std::size_t pos = 0;
    const std::int64_t v = std::stoll(raw, &pos);
    if (pos == raw.size()) {
      out.emplace_back(prefix, v);
      return;
    }
  } catch (...) {
  }
  try {
    std::size_t pos = 0;
    const double v = std::stod(raw, &pos);
    if (pos == raw.size()) {
      out.emplace_back(prefix, v);
      return;
    }
  } catch (...) {
  }
  out.emplace_back(prefix, raw);
}

// ---------------------------------------------------------------------------
// Small typed parameter map (for the declarative sensor configuration)
// ---------------------------------------------------------------------------
struct ParamVal {
  std::variant<bool, std::int64_t, double, std::string, std::vector<bool>,
               std::vector<std::int64_t>, std::vector<double>, std::vector<std::string>>
    value;

  template <typename T>
  T get() const
  {
    if constexpr (std::is_same_v<T, std::int32_t>) {
      if (const auto* p = std::get_if<std::int64_t>(&value)) return static_cast<std::int32_t>(*p);
      if (const auto* p = std::get_if<double>(&value)) return static_cast<std::int32_t>(*p);
      throw std::runtime_error("ParamVal: incompatible type (int32)");
    } else if constexpr (std::is_same_v<T, std::int64_t>) {
      if (const auto* p = std::get_if<std::int64_t>(&value)) return *p;
      if (const auto* p = std::get_if<double>(&value)) return static_cast<std::int64_t>(*p);
      throw std::runtime_error("ParamVal: incompatible type (int64)");
    } else if constexpr (std::is_same_v<T, double>) {
      if (const auto* p = std::get_if<double>(&value)) return *p;
      if (const auto* p = std::get_if<std::int64_t>(&value)) return static_cast<double>(*p);
      throw std::runtime_error("ParamVal: incompatible type (double)");
    } else if constexpr (std::is_same_v<T, std::string>) {
      if (const auto* p = std::get_if<std::string>(&value)) return *p;
      throw std::runtime_error("ParamVal: incompatible type (string)");
    } else {
      if (const auto* p = std::get_if<T>(&value)) return *p;
      throw std::runtime_error("ParamVal: incompatible type (generic)");
    }
  }
};

using ParamMap = std::map<std::string, ParamVal>;

inline void parse_param_map_impl(
  const YAML::Node& node, const std::string& prefix, ParamMap& out)
{
  if (node.IsMap()) {
    for (const auto& kv : node) {
      const std::string key = kv.first.as<std::string>();
      const std::string full = prefix.empty() ? key : prefix + "." + key;
      parse_param_map_impl(kv.second, full, out);
    }
    return;
  }
  if (prefix.empty()) return;
  if (node.IsSequence()) {
    bool all_bool = true, all_int = true, all_num = true;
    for (const auto& e : node) {
      if (!e.IsScalar()) {
        all_bool = all_int = all_num = false;
        continue;
      }
      const std::string raw = e.Scalar();
      if (raw != "true" && raw != "false" && raw != "True" && raw != "False") all_bool = false;
      try {
        std::size_t pos = 0;
        std::stoll(raw, &pos);
        if (pos != raw.size()) all_int = false;
      } catch (...) {
        all_int = false;
      }
      try {
        std::size_t pos = 0;
        std::stod(raw, &pos);
        if (pos != raw.size()) all_num = false;
      } catch (...) {
        all_num = false;
      }
    }
    if (all_bool) {
      std::vector<bool> vec;
      for (const auto& e : node) vec.push_back(e.as<bool>());
      out[prefix].value = vec;
    } else if (all_int) {
      std::vector<std::int64_t> vec;
      for (const auto& e : node) vec.push_back(e.as<std::int64_t>());
      out[prefix].value = vec;
    } else if (all_num) {
      std::vector<double> vec;
      for (const auto& e : node) vec.push_back(e.as<double>());
      out[prefix].value = vec;
    } else {
      std::vector<std::string> vec;
      for (const auto& e : node) vec.push_back(e.as<std::string>());
      out[prefix].value = vec;
    }
    return;
  }
  if (!node.IsScalar()) return;
  const std::string raw = node.Scalar();
  if (raw == "true" || raw == "True" || raw == "false" || raw == "False") {
    out[prefix].value = node.as<bool>();
    return;
  }
  try {
    std::size_t pos = 0;
    const std::int64_t i = std::stoll(raw, &pos);
    if (pos == raw.size()) {
      out[prefix].value = i;
      return;
    }
  } catch (...) {
  }
  try {
    std::size_t pos = 0;
    const double d = std::stod(raw, &pos);
    if (pos == raw.size()) {
      out[prefix].value = d;
      return;
    }
  } catch (...) {
  }
  out[prefix].value = raw;
}

inline ParamMap parse_param_map(const YAML::Node& node)
{
  ParamMap out;
  parse_param_map_impl(node, "", out);
  return out;
}

}  // namespace farlio_ros1
