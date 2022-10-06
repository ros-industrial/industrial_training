#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import py_perception.srv
import time
from py_perception.srv import FilterCloud
from sensor_msgs.msg import PointCloud2

class FilterNode(Node):
    def __init__(self):
        super().__init__('filter_cloud', allow_undeclared_parameters = False,
                         automatically_declare_parameters_from_overrides = True)

        # SET UP SERVICE CLIENT
        self.client = self.create_client(FilterCloud, 'filter_cloud')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.pcdfilename = self.get_parameter_or('pcdfilename', rclpy.parameter.Parameter('', rclpy.parameter.Parameter.Type.STRING, '')).value
        self.last_cloud = None

        # PUBLISHERS
        self.voxel_pub = self.create_publisher(PointCloud2, '/perception_voxelGrid', 1)

        self.call_filters()

    def call_filters(self):
        while rclpy.ok():
            self.voxel_filter()

            time.sleep(3.0) # sleep for 3 seconds

    def voxel_filter(self):
        # =======================
        # VOXEL GRID FILTER
        # =======================
        pass

    def passthrough_filter(self):
        # =======================
        # PASSTHROUGH FILTER
        # =======================
        pass

    def plane_segmentation(self):
        # =======================
        # PLANE SEGMENTATION
        # =======================
        pass

    def cluster_extraction(self):
        # =======================
        # CLUSTER EXTRACTION
        # =======================
        pass


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(FilterNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
