#!/usr/bin/env python

import rospy
import py_perception.srv
from sensor_msgs.msg import PointCloud2

if __name__ == '__main__':
    try:
        rospy.init_node('filter_cloud', anonymous=True)
        rospy.wait_for_service('filter_cloud')

        # =======================
        # VOXEL GRID FILTER
        # =======================

        srvp = rospy.ServiceProxy('filter_cloud', py_perception.srv.FilterCloud)
        req = py_perception.srv.FilterCloudRequest()
        req.pcdfilename = rospy.get_param('~pcdfilename', '')
        req.operation = py_perception.srv.FilterCloudRequest.VOXELGRID

        # FROM THE SERVICE, ASSIGN POINTS 
        req.input_cloud = PointCloud2()

        # ERROR HANDLING
        if req.pcdfilename == '':
            raise Exception('No file parameter found')

        # PACKAGE THE FILTERED POINTCLOUD2 TO BE PUBLISHED
        res_voxel = srvp(req)
        print('response received')
        if not res_voxel.success:
            raise Exception('Unsuccessful voxel grid filter operation')

        # PUBLISH VOXEL FILTERED POINTCLOUD2
        pub = rospy.Publisher('/perception_voxelGrid', PointCloud2, queue_size=1, latch=True)
        pub.publish(res_voxel.output_cloud)
        print("published: voxel grid filter response")

        # =======================
        # PASSTHROUGH FILTER
        # =======================

        srvp = rospy.ServiceProxy('filter_cloud', py_perception.srv.FilterCloud)
        req = py_perception.srv.FilterCloudRequest()
        req.pcdfilename = ''
        req.operation = py_perception.srv.FilterCloudRequest.PASSTHROUGH
        # FROM THE SERVICE, ASSIGN POINTS
        req.input_cloud = res_voxel.output_cloud

        # PACKAGE THE FILTERED POINTCLOUD2 TO BE PUBLISHED
        res_pass = srvp(req)
        print('response received')
        if not res_voxel.success:
            raise Exception('Unsuccessful pass through filter operation')

        # PUBLISH PASSTHROUGH FILTERED POINTCLOUD2
        pub = rospy.Publisher('/perception_passThrough', PointCloud2, queue_size=1, latch=True)
        pub.publish(res_pass.output_cloud)
        print("published: pass through filter response")

        # =======================
        # PLANE SEGMENTATION
        # =======================

        srvp = rospy.ServiceProxy('filter_cloud', py_perception.srv.FilterCloud)
        req = py_perception.srv.FilterCloudRequest()
        req.pcdfilename = ''
        req.operation = py_perception.srv.FilterCloudRequest.PLANESEGMENTATION
        # FROM THE SERVICE, ASSIGN POINTS
        req.input_cloud = res_pass.output_cloud

        # PACKAGE THE FILTERED POINTCLOUD2 TO BE PUBLISHED
        res_seg = srvp(req)
        print('response received')
        if not res_voxel.success:
            raise Exception('Unsuccessful plane segmentation operation')

        # PUBLISH PLANESEGMENTATION FILTERED POINTCLOUD2
        pub = rospy.Publisher('/perception_planeSegmentation', PointCloud2, queue_size=1, latch=True)
        pub.publish(res_seg.output_cloud)
        print("published: plane segmentation filter response")

        # =======================
        # CLUSTER EXTRACTION
        # =======================

        srvp = rospy.ServiceProxy('filter_cloud', py_perception.srv.FilterCloud)
        req = py_perception.srv.FilterCloudRequest()
        req.pcdfilename = ''
        req.operation = py_perception.srv.FilterCloudRequest.CLUSTEREXTRACTION
        # FROM THE SERVICE, ASSIGN POINTS
        req.input_cloud = res_seg.output_cloud

        # PACKAGE THE FILTERED POINTCLOUD2 TO BE PUBLISHED
        res_cluster = srvp(req)
        print('response received')
        if not res_voxel.success:
            raise Exception('Unsuccessful cluster extraction operation')

        # PUBLISH CLUSTEREXTRACTION FILTERED POINTCLOUD2
        pub = rospy.Publisher('/perception_clusterExtraction', PointCloud2, queue_size=1, latch=True)
        pub.publish(res_cluster.output_cloud)
        print("published: cluster extraction filter response")

        rospy.spin()
    except Exception as e:
        print("Service call failed: %s" % str(e))
