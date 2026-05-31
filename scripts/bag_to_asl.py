#!/usr/bin/env python3

import argparse
import os

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image, Imu


def stamp_to_ns(stamp):
    return stamp.sec * 1000000000 + stamp.nanosec


class AslDumper(Node):
    def __init__(self, root_folder, image_topic, compressed_image_topic, imu_topic):
        super().__init__("asl_format")
        self.bridge = CvBridge()

        os.makedirs(root_folder, exist_ok=True)

        img_folder = os.path.join(root_folder, "cam0")
        self.img_data_folder = os.path.join(img_folder, "data")
        os.makedirs(self.img_data_folder, exist_ok=True)
        self.img_file = open(os.path.join(img_folder, "data.csv"), "w")
        self.img_file.write("#timestamp [ns],filename\n")

        imu_folder = os.path.join(root_folder, "imu0")
        os.makedirs(imu_folder, exist_ok=True)
        self.imu_file = open(os.path.join(imu_folder, "data.csv"), "w")
        self.imu_file.write(
            "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],"
            "w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n"
        )

        self.create_subscription(CompressedImage, compressed_image_topic, self.compressed_image_sub, 10)
        self.create_subscription(Image, image_topic, self.image_sub, 10)
        self.create_subscription(Imu, imu_topic, self.imu_sub, 100)

    def close(self):
        self.imu_file.close()
        self.img_file.close()

    def imu_sub(self, imu_msg):
        stamp = stamp_to_ns(imu_msg.header.stamp)
        acc = imu_msg.linear_acceleration
        ang_vel = imu_msg.angular_velocity
        self.imu_file.write(
            f"{stamp},{ang_vel.x},{ang_vel.y},{ang_vel.z},{acc.x},{acc.y},{acc.z}\n"
        )

    def compressed_image_sub(self, img_msg):
        stamp = stamp_to_ns(img_msg.header.stamp)
        np_arr = np.frombuffer(img_msg.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.write_image(stamp, image_np)

    def image_sub(self, img_msg):
        stamp = stamp_to_ns(img_msg.header.stamp)
        cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="passthrough")
        self.write_image(stamp, cv_image)

    def write_image(self, stamp, image):
        filename = f"{stamp}.png"
        cv2.imwrite(os.path.join(self.img_data_folder, filename), image)
        self.img_file.write(f"{stamp},{filename}\n")


def main():
    parser = argparse.ArgumentParser(description="Convert ROS2 GoPro topics to Euroc/ASL files.")
    parser.add_argument("--base_dir", required=True, help="Output mav0-style directory.")
    parser.add_argument("--image_topic", default="/gopro/image_raw")
    parser.add_argument("--compressed_image_topic", default="/gopro/image_raw/compressed")
    parser.add_argument("--imu_topic", default="/gopro/imu")
    args = parser.parse_args()

    rclpy.init()
    node = AslDumper(args.base_dir, args.image_topic, args.compressed_image_topic, args.imu_topic)
    try:
        rclpy.spin(node)
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
