// Licensed under the Apache License, Version 2.0.
//
// ROS 1 equivalent of TUMFTM/TAM__common :: ros2_watchdog_cpp TopicWatchdog.
// Tracks the age of subscribed topics on the 100 Hz model-update timer and
// invokes the timeout callback on state transitions (exactly like upstream).
#pragma once

#include <ros/ros.h>

#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <string>

namespace tam::core {

class TopicWatchdog {
 public:
  using Callback = std::function<void(bool is_timeout, double elapsed_seconds)>;

  void add(const std::string& key, double timeout_seconds, Callback cb)
  {
    Entry entry;
    entry.timeout = timeout_seconds;
    entry.last_stamp = ros::Time::now();
    entry.callback = std::move(cb);
    entries_[key] = std::move(entry);
  }

  void update(const std::string& key)
  {
    auto it = entries_.find(key);
    if (it == entries_.end()) return;
    it->second.last_stamp = ros::Time::now();
    if (it->second.timed_out) {
      it->second.timed_out = false;
      if (it->second.callback) it->second.callback(false, 0.0);
    }
  }

  void check_timeouts()
  {
    const ros::Time now = ros::Time::now();
    for (auto& [key, entry] : entries_) {
      const double elapsed = (now - entry.last_stamp).toSec();
      if (!entry.timed_out && elapsed > entry.timeout) {
        entry.timed_out = true;
        if (entry.callback) entry.callback(true, elapsed);
      }
    }
  }

  /** @brief Time since the last message on a key (seconds, inf if unknown). */
  double age(const std::string& key) const
  {
    auto it = entries_.find(key);
    if (it == entries_.end()) return std::numeric_limits<double>::infinity();
    return (ros::Time::now() - it->second.last_stamp).toSec();
  }

 private:
  struct Entry {
    double timeout{0.5};
    ros::Time last_stamp{};
    bool timed_out{false};
    Callback callback{};
  };
  std::map<std::string, Entry> entries_;
};

}  // namespace tam::core
