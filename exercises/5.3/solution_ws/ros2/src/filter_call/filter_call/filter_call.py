#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import time
import py_perception.srv
from py_perception.srv import FilterCloud
from sensor_msgs.msg import PointCloud2

class FilterNode(Node):
    def __init__(self):
        super().__init__('filter_cloud', allow_undeclared_parameters = True,
                         automatically_declare_parameters_from_overrides = True)
        self.client = self.create_client(FilterCloud, 'filter_cloud')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.pcdfilename = self.get_parameter_or('pcdfilename', rclpy.parameter.Parameter('', rclpy.parameter.Parameter.Type.STRING, '')).value
        self.last_cloud = None

        # PUBLISHERS
        self.voxel_pub = self.create_publisher(PointCloud2, '/perception_voxelGrid', 1)
        self.pass_pub = self.create_publisher(PointCloud2, '/perception_passthrough', 1)
        self.plane_pub = self.create_publisher(PointCloud2, '/perception_planeSegmentation', 1)
        self.cluster_pub = self.create_publisher(PointCloud2, '/perception_clusterExtraction', 1)

        self.call_filters()

    def call_filters(self):
        while rclpy.ok():
            self.voxel_filter()
            self.passthrough_filter()
            self.plane_segmentation()
            self.cluster_extraction()
            time.sleep(3.0)

    def voxel_filter(self):
        # =======================
        # VOXEL GRID FILTER
        # =======================
        req = FilterCloud.Request()
        req.pcdfilename = self.pcdfilename
        req.operation = py_perception.srv.FilterCloud.Request.VOXELGRID

        # FROM THE SERVICE, ASSIGN POINTS
        req.input_cloud = PointCloud2()

        # ERROR HANDLING
        if req.pcdfilename == '':
            self.get_logger().error('No file parameter found')
            return

        # PACKAGE THE FILTERED POINTCLOUD2 TO BE PUBLISHED
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res_voxel = future.result()
        if not res_voxel.success:
            self.get_logger().error('Unsuccessful voxel grid filter operation')
            return

        # PUBLISH VOXEL FILTERED POINTCLOUD2
        self.voxel_pub.publish(res_voxel.output_cloud)
        self.last_cloud = res_voxel
        self.get_logger().info("published: voxel grid filter response")

    def passthrough_filter(self):
        # =======================
        # PASSTHROUGH FILTER
        # =======================
        req = FilterCloud.Request()
        req.pcdfilename = ''
        req.operation = py_perception.srv.FilterCloud.Request.PASSTHROUGH
        # FROM THE SERVICE, ASSIGN POINTS
        req.input_cloud = self.last_cloud.output_cloud

        # PACKAGE THE FILTERED POINTCLOUD2 TO BE PUBLISHED
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res_pass = future.result()
        if not res_pass.success:
            self.get_logger().error('Unsuccessful passthrough filter operation')
            return

        # PUBLISH PASSTHROUGH FILTERED POINTCLOUD2
        self.pass_pub.publish(res_pass.output_cloud)
        self.last_cloud = res_pass
        self.get_logger().info("published: passthrough filter response")

    def plane_segmentation(self):
        # =======================
        # PLANE SEGMENTATION
        # =======================
        req = FilterCloud.Request()
        req.pcdfilename = ''
        req.operation = py_perception.srv.FilterCloud.Request.PLANESEGMENTATION
        # FROM THE SERVICE, ASSIGN POINTS
        req.input_cloud = self.last_cloud.output_cloud

        # PACKAGE THE FILTERED POINTCLOUD2 TO BE PUBLISHED
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res_seg = future.result()
        if not res_seg.success:
            self.get_logger().error('Unsuccessful plane segmentation operation')
            return

        # PUBLISH PLANESEGMENTATION FILTERED POINTCLOUD2
        self.plane_pub.publish(res_seg.output_cloud)
        self.last_cloud = res_seg
        self.get_logger().info("published: plane segmentation filter response")

    def cluster_extraction(self):
        # =======================
        # CLUSTER EXTRACTION
        # =======================
        req = FilterCloud.Request()
        req.pcdfilename = ''
        req.operation = py_perception.srv.FilterCloud.Request.CLUSTEREXTRACTION
        # FROM THE SERVICE, ASSIGN POINTS
        req.input_cloud = self.last_cloud.output_cloud

        # PACKAGE THE FILTERED POINTCLOUD2 TO BE PUBLISHED
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res_cluster = future.result()
        if not res_cluster.success:
            self.get_logger().error('Unsuccessful cluster extraction operation')
            return

        # PUBLISH CLUSTEREXTRACTION FILTERED POINTCLOUD2
        self.cluster_pub.publish(res_cluster.output_cloud)
        self.last_cloud = res_cluster
        self.get_logger().info("published: cluster extraction filter response")


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(FilterNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
