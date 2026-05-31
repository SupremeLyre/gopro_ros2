//
// Created by bjoshi on 10/29/20.
//

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>

#include <filesystem>
#include <iostream>

#include "ImuExtractor.h"
#include "VideoExtractor.h"
#include "color_codes.h"
#include "ros2_compat.h"
#include "time_utils.h"

using namespace std;
namespace fs = std::filesystem;

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("gopro_to_rosbag");

  string gopro_video;
  string gopro_folder;
  string rosbag;

  gopro_video = node->declare_parameter<std::string>("gopro_video", "");
  gopro_folder = node->declare_parameter<std::string>("gopro_folder", "");
  rosbag = node->declare_parameter<std::string>("rosbag", "");

  bool is_gopro_video = !gopro_video.empty();
  bool is_gopro_folder = !gopro_folder.empty();

  if (!is_gopro_video && !is_gopro_folder) {
    ROS_FATAL("Please specify the gopro video or folder");
    rclcpp::shutdown();
    return 1;
  }

  if (rosbag.empty()) {
    ROS_FATAL_STREAM("No rosbag file specified");
    rclcpp::shutdown();
    return 1;
  }

  double scaling = node->declare_parameter<double>("scale", 1.0);

  bool compress_images = node->declare_parameter<bool>("compressed_image_format", true);

  bool grayscale = node->declare_parameter<bool>("grayscale", false);

  bool display_images = node->declare_parameter<bool>("display_images", false);

  bool multiple_files = node->declare_parameter<bool>("multiple_files", false);

  rosbag2_cpp::Writer bag;
  bag.open(rosbag);

  vector<fs::path> video_files;

  if (is_gopro_folder && multiple_files) {
    std::copy(fs::directory_iterator(gopro_folder),
              fs::directory_iterator(),
              std::back_inserter(video_files));
    std::sort(video_files.begin(), video_files.end());

  } else {
    video_files.push_back(fs::path(gopro_video));
  }

  auto end = std::remove_if(video_files.begin(), video_files.end(), [](const fs::path& p) {
    return p.extension() != ".MP4" || fs::is_directory(p);
  });
  video_files.erase(end, video_files.end());

  vector<uint64_t> start_stamps;
  vector<uint32_t> samples;
  std::deque<AcclMeasurement> accl_queue;
  std::deque<GyroMeasurement> gyro_queue;
  std::deque<MagMeasurement> magnetometer_queue;

  vector<uint64_t> image_stamps;

  bool has_magnetic_field_readings = false;
  bool has_gps_readings = false;
  for (uint32_t i = 0; i < video_files.size(); i++) {
    image_stamps.clear();

    ROS_WARN_STREAM("Opening Video File: " << video_files[i].filename().string());

    fs::path file = video_files[i];
    GoProImuExtractor imu_extractor(file.string());
    GoProVideoExtractor video_extractor(file.string(), scaling, true);

    if (i == 0 && imu_extractor.isSensorPresent(STR2FOURCC("MAGN"))) {
      has_magnetic_field_readings = true;
    }
    if (i == 0 && imu_extractor.isSensorPresent(STR2FOURCC("GPS9"))) {
      has_gps_readings = true;
    }
    
    imu_extractor.getPayloadStamps(STR2FOURCC("ACCL"), start_stamps, samples);
    ROS_INFO_STREAM("[ACCL] Payloads: " << start_stamps.size()
                                        << " Start stamp: " << start_stamps[0]
                                        << " End stamp: " << start_stamps[samples.size() - 1]
                                        << " Total Samples: " << samples.at(samples.size() - 1));
    imu_extractor.getPayloadStamps(STR2FOURCC("GYRO"), start_stamps, samples);
    ROS_INFO_STREAM("[GYRO] Payloads: " << start_stamps.size()
                                        << " Start stamp: " << start_stamps[0]
                                        << " End stamp: " << start_stamps[samples.size() - 1]
                                        << " Total Samples: " << samples.at(samples.size() - 1));
    imu_extractor.getPayloadStamps(STR2FOURCC("CORI"), start_stamps, samples);
    ROS_INFO_STREAM("[Image] Payloads: " << start_stamps.size()
                                         << " Start stamp: " << start_stamps[0]
                                         << " End stamp: " << start_stamps[samples.size() - 1]
                                         << " Total Samples: " << samples.at(samples.size() - 1));
    if (has_magnetic_field_readings) {
      imu_extractor.getPayloadStamps(STR2FOURCC("MAGN"), start_stamps, samples);
      ROS_INFO_STREAM("[MAGN] Payloads: " << start_stamps.size()
                                          << " Start stamp: " << start_stamps[0]
                                          << " End stamp: " << start_stamps[samples.size() - 1]
                                          << " Total Samples: " << samples.at(samples.size() - 1));
    }

    if (has_gps_readings) {
      imu_extractor.getPayloadStamps(STR2FOURCC("GPS9"), start_stamps, samples);
      ROS_INFO_STREAM("[GPS9] Payloads: " << start_stamps.size()
                                          << " Start stamp: " << start_stamps[0]
                                          << " End stamp: " << start_stamps[samples.size() - 1]
                                          << " Total Samples: " << samples.at(samples.size() - 1));
    }

    uint64_t accl_end_stamp = 0, gyro_end_stamp = 0;
    uint64_t video_end_stamp = 0;
    uint64_t magnetometer_end_stamp = 0;
    uint64_t gps_end_stamp = 0;

    if (i < video_files.size() - 1) {
      GoProImuExtractor imu_extractor_next(video_files[i + 1].string());
      accl_end_stamp = imu_extractor_next.getPayloadStartStamp(STR2FOURCC("ACCL"), 0);
      gyro_end_stamp = imu_extractor_next.getPayloadStartStamp(STR2FOURCC("GYRO"), 0);
      video_end_stamp = imu_extractor_next.getPayloadStartStamp(STR2FOURCC("CORI"), 0);
      if (has_magnetic_field_readings) {
        magnetometer_end_stamp = imu_extractor_next.getPayloadStartStamp(STR2FOURCC("MAGN"), 0);
      }
      if (has_gps_readings) {
        gps_end_stamp = imu_extractor_next.getPayloadStartStamp(STR2FOURCC("GPS9"), 0);
      }
    }

    imu_extractor.readImuData(accl_queue, gyro_queue, accl_end_stamp, gyro_end_stamp);
    imu_extractor.readMagnetometerData(magnetometer_queue, magnetometer_end_stamp);

    if (has_gps_readings) {
      imu_extractor.writeGpsData(bag, gps_end_stamp, "/gopro/gps");
    }

    uint32_t gpmf_frame_count = imu_extractor.getImageCount();
    uint32_t ffmpeg_frame_count = video_extractor.getFrameCount();
    if (gpmf_frame_count != ffmpeg_frame_count) {
      ROS_FATAL_STREAM("Video and metadata frame count do not match");
      rclcpp::shutdown();
      return 1;
    }

    uint64_t gpmf_video_time = imu_extractor.getVideoCreationTime();
    uint64_t ffmpeg_video_time = video_extractor.getVideoCreationTime();

    if (ffmpeg_video_time != gpmf_video_time) {
      ROS_FATAL_STREAM("Video creation time does not match");
      rclcpp::shutdown();
      return 1;
    }

    imu_extractor.getImageStamps(image_stamps, video_end_stamp);
    if (i != video_files.size() - 1 && image_stamps.size() != ffmpeg_frame_count) {
      ROS_FATAL_STREAM("ffmpeg and gpmf frame count does not match.");
      ROS_FATAL_STREAM(image_stamps.size() << " vs " << ffmpeg_frame_count);
      rclcpp::shutdown();
      return 1;
    }

    video_extractor.writeVideo(
        bag, "/gopro/image_raw", image_stamps, grayscale, compress_images, display_images);
  }

  ROS_INFO_STREAM("[ACCL] Payloads: " << accl_queue.size());
  ROS_INFO_STREAM("[GYRO] Payloads: " << gyro_queue.size());

  assert(accl_queue.size() == gyro_queue.size());

  double previous = accl_queue.front().timestamp_ * 1e-9;

  while (!accl_queue.empty() && !gyro_queue.empty()) {
    AcclMeasurement accl = accl_queue.front();
    GyroMeasurement gyro = gyro_queue.front();
    int64_t diff = accl.timestamp_ - gyro.timestamp_;

    Timestamp stamp;
    if (abs(diff) > 100000) {
      // I will need to handle this case more carefully
      ROS_WARN_STREAM(diff << " ns difference between gyro and accl");
      stamp = (Timestamp)(((double)accl.timestamp_ + (double)gyro.timestamp_) / 2.0);
    } else {
      stamp = accl.timestamp_;
    }

    double current = stamp * 1e-9;
    // assert(abs(current - previous) < 0.001);
    previous = current;
    auto ros_time = rclTimeFromNanoseconds(stamp);
    sensor_msgs::Imu imu_msg;
    std_msgs::Header header;
    header.stamp = stampFromNanoseconds(stamp);
    header.frame_id = "body";
    imu_msg.header = header;
    imu_msg.linear_acceleration.x = accl.data_.x();
    imu_msg.linear_acceleration.y = accl.data_.y();
    imu_msg.linear_acceleration.z = accl.data_.z();
    imu_msg.angular_velocity.x = gyro.data_.x();
    imu_msg.angular_velocity.y = gyro.data_.y();
    imu_msg.angular_velocity.z = gyro.data_.z();

    bag.write(imu_msg, "/gopro/imu", ros_time);

    accl_queue.pop_front();
    gyro_queue.pop_front();
  }

  while (!magnetometer_queue.empty()) {
    MagMeasurement mag = magnetometer_queue.front();
    Timestamp stamp = mag.timestamp_;

    auto ros_time = rclTimeFromNanoseconds(stamp);
    sensor_msgs::MagneticField magnetic_field_msg;
    std_msgs::Header header;
    header.stamp = stampFromNanoseconds(stamp);
    header.frame_id = "body";
    magnetic_field_msg.header = header;
    magnetic_field_msg.magnetic_field.x = mag.magfield_.x();
    magnetic_field_msg.magnetic_field.y = mag.magfield_.y();
    magnetic_field_msg.magnetic_field.z = mag.magfield_.z();

    bag.write(magnetic_field_msg, "/gopro/magnetic_field", ros_time);

    magnetometer_queue.pop_front();
  }

  bag.close();
  rclcpp::shutdown();
  return 0;
}
