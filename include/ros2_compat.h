#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <std_msgs/msg/header.hpp>

namespace sensor_msgs {
using CompressedImage = msg::CompressedImage;
using CompressedImagePtr = msg::CompressedImage::SharedPtr;
using Image = msg::Image;
using ImagePtr = msg::Image::SharedPtr;
using Imu = msg::Imu;
using MagneticField = msg::MagneticField;
using NavSatFix = msg::NavSatFix;
using NavSatStatus = msg::NavSatStatus;
}  // namespace sensor_msgs

namespace std_msgs {
using Header = msg::Header;
}  // namespace std_msgs

#define GOPRO_ROS_LOGGER rclcpp::get_logger("gopro_ros")

#define ROS_INFO_STREAM(args) RCLCPP_INFO_STREAM(GOPRO_ROS_LOGGER, args)
#define ROS_WARN_STREAM(args) RCLCPP_WARN_STREAM(GOPRO_ROS_LOGGER, args)
#define ROS_ERROR_STREAM(args) RCLCPP_ERROR_STREAM(GOPRO_ROS_LOGGER, args)
#define ROS_FATAL_STREAM(args) RCLCPP_FATAL_STREAM(GOPRO_ROS_LOGGER, args)
#define ROS_FATAL(args) RCLCPP_FATAL(GOPRO_ROS_LOGGER, args)
#define ROS_FATAL_STREAM_COND(cond, args) \
  do {                                    \
    if (cond) {                           \
      RCLCPP_FATAL_STREAM(GOPRO_ROS_LOGGER, args); \
    }                                     \
  } while (0)
