#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from cal_demo_motion.srv import MoveRobot
from sensor_msgs.msg import Image, CameraInfo

from ament_index_python.packages import get_package_share_directory

from os import listdir
from os.path import isfile, join

import yaml

import cv2
from cv_bridge import CvBridge

import time


def main():

    rclpy.init()

    node = Node("planning_client")

    client = node.create_client(MoveRobot, "move_robot")

    img_publisher = node.create_publisher(Image, "/camera/image_raw", 10)
    camera_info_pub = node.create_publisher(CameraInfo, "/camera/camera_info", 10)

    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')

    data_path = join(get_package_share_directory("cal_demo_motion"), "data")

    pose_files = [str(i) + ".yaml" for i in range(16)] # 16 data captures
    image_files = [str(i) + ".png" for i in range(16)]

    bridge = CvBridge()

    for pose_file, image_file in zip(pose_files, image_files):

        with open(join(data_path, "poses", pose_file), "r") as f:
            pose = yaml.safe_load(f)

        request = MoveRobot.Request()

        request.goal.header.frame_id = "base_link"
        request.goal.pose.orientation.w = pose['qw']
        request.goal.pose.orientation.x = pose['qx']
        request.goal.pose.orientation.y = pose['qy']
        request.goal.pose.orientation.z = pose['qz']
        request.goal.pose.position.x = pose['x']
        request.goal.pose.position.y = pose['y']
        request.goal.pose.position.z = pose['z']

        node.get_logger().info("Sending request!")

        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)

        response = future.result()
        node.get_logger().info(response.message)

        img = cv2.imread(join(data_path, "images", image_file))

        img_msg = bridge.cv2_to_imgmsg(img, "bgr8")
        img_msg.header.frame_id = "camera"

        info = CameraInfo()

        info.height = 1200
        info.width = 1600

        info.k = [1352.02747, 0.0, 789.67065,
                   0.0, 1356.14287, 627.2995,
                   0.0, 0.0, 1.0]
        
        info.d = [0.0] * 5

        # [ k | 0] for mono
        info.p = [1352.02747, 0.0, 789.67065, 0.0,
                  0.0, 1356.14287, 627.2995, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        
        for i in range(30):

            img_msg.header.stamp = node.get_clock().now().to_msg()
            info.header = img_msg.header

            img_publisher.publish(img_msg)

            camera_info_pub.publish(info)
            time.sleep(0.03)
        
        input("Hit enter to continue...")
    
if __name__ == "__main__":

    main()