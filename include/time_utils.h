#pragma once

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/time.hpp>

inline builtin_interfaces::msg::Time stampFromNanoseconds(uint64_t stamp) {
  builtin_interfaces::msg::Time time_msg;
  time_msg.sec = static_cast<int32_t>(stamp / 1000000000ULL);
  time_msg.nanosec = static_cast<uint32_t>(stamp % 1000000000ULL);
  return time_msg;
}

inline rclcpp::Time rclTimeFromNanoseconds(uint64_t stamp) {
  return rclcpp::Time(stampFromNanoseconds(stamp));
}
