// Licensed under the Apache License, Version 2.0.
//
// ROS 1 port of TUMFTM/tsl :: tsl_ros2_publisher_cpp.
//
// The TSL (time series logging) debug interface is preserved: a definition
// message is latched whenever the set of logged signals changes, and value
// messages are published on every trigger. The ROS 1 messages live in
// farlio_msgs (TSLDefinition / TSLValues) and mirror the upstream tsl_msgs layout
// (typed name/value arrays).
#pragma once

#include <ros/ros.h>

#include <cstdint>
#include <string>
#include <unordered_map>
#include <variant>
#include <vector>

#include <farlio_msgs/TSLDefinition.h>
#include <farlio_msgs/TSLValues.h>

#include "tsl_logger_cpp/base.hpp"
#include "tsl_logger_cpp/value_logger.hpp"

namespace farlio_ros1 {

class TslPublisher {
 public:
  TslPublisher(ros::NodeHandle& nh, tam::tsl::LoggerAccessInterface::SharedPtr logger)
      : logger_(std::move(logger))
  {
    def_pub_ = nh.advertise<farlio_msgs::TSLDefinition>("tsl_definition", 1, true);
    val_pub_ = nh.advertise<farlio_msgs::TSLValues>("tsl_values", 5);
  }

  void trigger()
  {
    if (!logger_) return;
    const auto& data = logger_->get_data();
    if (data.empty()) return;

    const std::uint64_t hash = compute_hash(data);
    if (hash != definition_hash_) {
      publish_definition(data, hash);
      definition_hash_ = hash;
    }
    publish_values(data);
  }

 private:
  using Key = std::string;

  struct Schema {
    std::vector<Key> bools, int8s, uint8s, int16s, uint16s, int32s, uint32s, int64s, uint64s,
      float32s, float64s;
  };

  static Schema make_schema(const tam::tsl::data_map_t& data)
  {
    Schema s;
    for (const auto& [name, value] : data) {
      std::visit(
        [&](const auto& v) {
          using T = std::decay_t<decltype(v)>;
          if constexpr (std::is_same_v<T, bool>)
            s.bools.push_back(name);
          else if constexpr (std::is_same_v<T, std::int8_t>)
            s.int8s.push_back(name);
          else if constexpr (std::is_same_v<T, std::uint8_t>)
            s.uint8s.push_back(name);
          else if constexpr (std::is_same_v<T, std::int16_t>)
            s.int16s.push_back(name);
          else if constexpr (std::is_same_v<T, std::uint16_t>)
            s.uint16s.push_back(name);
          else if constexpr (std::is_same_v<T, std::int32_t>)
            s.int32s.push_back(name);
          else if constexpr (std::is_same_v<T, std::uint32_t>)
            s.uint32s.push_back(name);
          else if constexpr (std::is_same_v<T, std::int64_t>)
            s.int64s.push_back(name);
          else if constexpr (std::is_same_v<T, std::uint64_t>)
            s.uint64s.push_back(name);
          else if constexpr (std::is_same_v<T, float>)
            s.float32s.push_back(name);
          else if constexpr (std::is_same_v<T, double>)
            s.float64s.push_back(name);
        },
        value);
    }
    return s;
  }

  static std::uint64_t compute_hash(const tam::tsl::data_map_t& data)
  {
    std::uint64_t hash = 1469598103934665603ULL;
    auto mix = [&hash](const std::string& s) {
      for (const char c : s) {
        hash ^= static_cast<std::uint8_t>(c);
        hash *= 1099511628211ULL;
      }
    };
    Schema s = make_schema(data);
    for (const auto* group : {&s.bools, &s.int8s, &s.uint8s, &s.int16s, &s.uint16s, &s.int32s,
                              &s.uint32s, &s.int64s, &s.uint64s, &s.float32s, &s.float64s}) {
      for (const auto& n : *group) {
        mix(n);
        mix("|");
      }
      mix("#");
    }
    return hash;
  }

  void publish_definition(const tam::tsl::data_map_t& data, std::uint64_t hash)
  {
    Schema s = make_schema(data);
    farlio_msgs::TSLDefinition msg;
    msg.header.stamp = ros::Time::now();
    msg.definition_hash = hash;
    msg.bool_names = s.bools;
    msg.int8_names = s.int8s;
    msg.uint8_names = s.uint8s;
    msg.int16_names = s.int16s;
    msg.uint16_names = s.uint16s;
    msg.int32_names = s.int32s;
    msg.uint32_names = s.uint32s;
    msg.int64_names = s.int64s;
    msg.uint64_names = s.uint64s;
    msg.float32_names = s.float32s;
    msg.float64_names = s.float64s;
    def_pub_.publish(msg);
  }

  void publish_values(const tam::tsl::data_map_t& data)
  {
    Schema s = make_schema(data);
    farlio_msgs::TSLValues msg;
    msg.header.stamp = ros::Time::now();
    msg.definition_hash = definition_hash_;
    for (const auto& n : s.bools) msg.bool_values.push_back(std::get<bool>(data.at(n)));
    for (const auto& n : s.int8s) msg.int8_values.push_back(std::get<std::int8_t>(data.at(n)));
    for (const auto& n : s.uint8s) msg.uint8_values.push_back(std::get<std::uint8_t>(data.at(n)));
    for (const auto& n : s.int16s) msg.int16_values.push_back(std::get<std::int16_t>(data.at(n)));
    for (const auto& n : s.uint16s) msg.uint16_values.push_back(std::get<std::uint16_t>(data.at(n)));
    for (const auto& n : s.int32s) msg.int32_values.push_back(std::get<std::int32_t>(data.at(n)));
    for (const auto& n : s.uint32s) msg.uint32_values.push_back(std::get<std::uint32_t>(data.at(n)));
    for (const auto& n : s.int64s) msg.int64_values.push_back(std::get<std::int64_t>(data.at(n)));
    for (const auto& n : s.uint64s) msg.uint64_values.push_back(std::get<std::uint64_t>(data.at(n)));
    for (const auto& n : s.float32s) msg.float32_values.push_back(std::get<float>(data.at(n)));
    for (const auto& n : s.float64s) msg.float64_values.push_back(std::get<double>(data.at(n)));
    val_pub_.publish(msg);
  }

  tam::tsl::LoggerAccessInterface::SharedPtr logger_;
  ros::Publisher def_pub_, val_pub_;
  std::uint64_t definition_hash_{0};
};

}  // namespace farlio_ros1
