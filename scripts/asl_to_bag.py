#!/usr/bin/env python3

import argparse
import csv
import glob
import os

from builtin_interfaces.msg import Time
import cv2
from cv_bridge import CvBridge
from rclpy.serialization import serialize_message
import rosbag2_py
from sensor_msgs.msg import Image, Imu
from tqdm import tqdm


def ns_to_time(stamp_ns):
    stamp_ns = int(stamp_ns)
    stamp = Time()
    stamp.sec = int(stamp_ns // 1000000000)
    stamp.nanosec = stamp_ns % 1000000000
    return stamp


def create_writer(bag_uri):
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=bag_uri, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    return writer


def create_topic(writer, name, msg_type):
    writer.create_topic(rosbag2_py.TopicMetadata(0, name, msg_type, "cdr"))


def write_images(writer, base_dir, topic, grayscale):
    bridge = CvBridge()
    cam0_folder = os.path.join(base_dir, "cam0", "data")
    files = sorted(glob.glob(os.path.join(cam0_folder, "*")))

    create_topic(writer, topic, "sensor_msgs/msg/Image")

    for file_path in tqdm(files, desc="images"):
        stamp_ns = int(os.path.splitext(os.path.basename(file_path))[0])
        image = cv2.imread(file_path, cv2.IMREAD_COLOR)
        if image is None:
            continue

        encoding = "bgr8"
        if grayscale:
            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            encoding = "mono8"

        msg = bridge.cv2_to_imgmsg(image, encoding=encoding)
        msg.header.stamp = ns_to_time(stamp_ns)
        msg.header.frame_id = "gopro"
        writer.write(topic, serialize_message(msg), stamp_ns)


def write_imu(writer, base_dir, topic):
    imu_path = os.path.join(base_dir, "imu0", "data.csv")
    if not os.path.exists(imu_path):
        return

    create_topic(writer, topic, "sensor_msgs/msg/Imu")

    with open(imu_path, newline="") as imu_file:
        reader = csv.reader(row for row in imu_file if not row.startswith("#"))
        for row in tqdm(reader, desc="imu"):
            if len(row) < 7:
                continue

            stamp_ns = int(row[0])
            imu = Imu()
            imu.header.stamp = ns_to_time(stamp_ns)
            imu.header.frame_id = "gopro"
            imu.angular_velocity.x = float(row[1])
            imu.angular_velocity.y = float(row[2])
            imu.angular_velocity.z = float(row[3])
            imu.linear_acceleration.x = float(row[4])
            imu.linear_acceleration.y = float(row[5])
            imu.linear_acceleration.z = float(row[6])
            writer.write(topic, serialize_message(imu), stamp_ns)


def main():
    parser = argparse.ArgumentParser(description="Convert Euroc/ASL files to a ROS2 bag.")
    parser.add_argument("--base_dir", required=True, help="Input mav0 directory.")
    parser.add_argument("--bag", required=True, help="Output ROS2 bag directory.")
    parser.add_argument("--image_topic", default="/cam0/image_raw")
    parser.add_argument("--imu_topic", default="/imu0")
    parser.add_argument("--no_imu", action="store_true", help="Skip imu0/data.csv.")
    parser.add_argument("--grayscale", action="store_true", help="Write images as mono8.")
    args = parser.parse_args()

    writer = create_writer(args.bag)
    write_images(writer, args.base_dir, args.image_topic, args.grayscale)
    if not args.no_imu:
        write_imu(writer, args.base_dir, args.imu_topic)
    writer.close()


if __name__ == "__main__":
    main()
