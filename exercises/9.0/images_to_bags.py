import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message

import rosbag2_py

from sensor_msgs.msg import Image

import sys
from os import listdir
from os.path import isfile, join

import cv2

from cv_bridge import CvBridge
import time

'''def main():

    rclpy.init()

    node = Node("convert")

    from os import listdir
    from os.path import isfile, join
    onlyfiles = ["left-{}.png".format(str(i).zfill(4)) for i in range(92)]
    onlyfiles.sort()

    publisher = node.create_publisher(Image, "camera/image_raw", 10)

    bridge = CvBridge()
    for file in onlyfiles:
        print(file)
        if file.endswith(".png"):
            img = cv2.imread("camera1_cal_data/" + file)

            img_msg = bridge.cv2_to_imgmsg(img, "bgr8")
            img_msg.header.frame_id = "camera"
            img_msg.header.stamp = node.get_clock().now().to_msg()

            publisher.publish(img_msg)

            time.sleep(0.3)'''
            
def main():

    rclpy.init()

    node = Node("images_to_bag_converter")
    
    num_files = [92, 75]

    for i in range(2):
    
        writer = rosbag2_py.SequentialWriter()
        
        storage_options = rosbag2_py._storage.StorageOptions(
                uri='intrinsics_rosbag{}'.format(i),
                storage_id='sqlite3')
                
        converter_options = rosbag2_py._storage.ConverterOptions('', '')
        
        writer.open(storage_options, converter_options)
        
        topic_info = rosbag2_py._storage.TopicMetadata(
            name='camera/image_raw',
            type='sensor_msgs/msg/Image',
            serialization_format='cdr')
            
        writer.create_topic(topic_info)
        
        onlyfiles = ["left-{}.png".format(str(i).zfill(4)) for i in range(num_files[i])]
        onlyfiles.sort()


        bridge = CvBridge()
        for file in onlyfiles:
            if file.endswith(".png"):
                img = cv2.imread("camera{}_cal_data/".format(i) + file)

                img_msg = bridge.cv2_to_imgmsg(img, "bgr8")
                img_msg.header.frame_id = "camera"
                img_msg.header.stamp = node.get_clock().now().to_msg()
                
                writer.write(
                    'camera/image_raw',
                    serialize_message(img_msg),
                    node.get_clock().now().nanoseconds)

                time.sleep(0.3)
            

if __name__ == "__main__":
    main()
