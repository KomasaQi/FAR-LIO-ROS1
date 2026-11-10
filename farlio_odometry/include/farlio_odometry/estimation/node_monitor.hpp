// Licensed under the Apache License, Version 2.0.
//
// ROS 1 equivalent of TUMFTM/TAM__common :: ros2_watchdog_cpp NodeMonitor.
// Publishes diagnostic_msgs/DiagnosticArray on "<topic>_status", with one
// status per monitored name (node name by default), worst-key error level,
// "causing_key" and arbitrary key/value diagnostics - same semantics as upstream.
#pragma once

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>

#include <map>
#include <string>
#include <vector>

#include "tum_types_cpp/common.hpp"

namespace tam::core {

class NodeMonitor {
 public:
  NodeMonitor(ros::NodeHandle& nh, const std::string& node_name, const std::string& custom_topic = "")
      : nh_(nh), node_name_(node_name), automatic_causing_key_(true)
  {
    std::string topic = custom_topic.empty() ? std::string("/core/orchestration/") + node_name
                                             : custom_topic;
    if (!topic.empty() && topic.back() == '/') topic.pop_back();
    pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>(topic + "_status", 10, true);
    stati_[node_name_] = Status{};
    set_message("Initializing");
    set_error_lvl("startup", tam::types::ErrorLvl::ERROR);
  }

  void set_error_lvl(const std::string& key, const tam::types::ErrorLvl& lvl)
  {
    stati_[node_name_].error_items[key] = lvl;
  }

  void set_message(const std::string& message) { stati_[node_name_].message = message; }

  void initialization_finished()
  {
    set_error_lvl("startup", tam::types::ErrorLvl::OK);
    set_message("Initialized");
  }

  void report_value(const std::string& name, const std::string& value)
  {
    stati_[node_name_].key_value_map[name] = value;
  }
  void report_value(const std::string& name, const int value)
  {
    report_value(name, std::to_string(value));
  }
  void report_value(const std::string& name, const double value)
  {
    report_value(name, std::to_string(value));
  }

  void update(const std_msgs::Header& header)
  {
    diagnostic_msgs::DiagnosticArray msg;
    msg.header = header;
    for (auto& [name, status] : stati_) {
      diagnostic_msgs::DiagnosticStatus status_msg;
      status_msg.name = name;
      status_msg.message = status.message;
      status_msg.level = static_cast<int8_t>(worst_level(name));
      status_msg.values = key_values(name);
      if (name == node_name_)
        msg.status.insert(msg.status.begin(), status_msg);
      else
        msg.status.push_back(status_msg);
    }
    pub_.publish(msg);
  }

  void update()
  {
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    update(header);
  }

 private:
  struct Status {
    std::map<std::string, tam::types::ErrorLvl> error_items{};
    std::map<std::string, std::string> key_value_map{};
    std::string message{};
  };

  tam::types::ErrorLvl worst_level(const std::string& name)
  {
    auto it = stati_.find(name);
    if (it == stati_.end() || it->second.error_items.empty()) return tam::types::ErrorLvl::OK;
    auto worst = it->second.error_items.begin();
    for (auto it2 = std::next(it->second.error_items.begin()); it2 != it->second.error_items.end();
         ++it2) {
      if (static_cast<int>(it2->second) > static_cast<int>(worst->second)) worst = it2;
    }
    if (automatic_causing_key_) report_value("causing_key", worst->first);
    return worst->second;
  }

  std::vector<diagnostic_msgs::KeyValue> key_values(const std::string& name)
  {
    std::vector<diagnostic_msgs::KeyValue> out;
    auto it = stati_.find(name);
    if (it == stati_.end()) return out;
    out.reserve(it->second.key_value_map.size() + it->second.error_items.size());
    for (const auto& [key, value] : it->second.key_value_map) {
      diagnostic_msgs::KeyValue kv;
      kv.key = key;
      kv.value = value;
      out.push_back(kv);
    }
    for (const auto& [key, level] : it->second.error_items) {
      diagnostic_msgs::KeyValue kv;
      kv.key = "level/" + key;
      kv.value = std::to_string(static_cast<int>(level));
      out.push_back(kv);
    }
    return out;
  }

  ros::NodeHandle& nh_;
  std::string node_name_;
  bool automatic_causing_key_;
  ros::Publisher pub_;
  std::map<std::string, Status> stati_;
};

}  // namespace tam::core
