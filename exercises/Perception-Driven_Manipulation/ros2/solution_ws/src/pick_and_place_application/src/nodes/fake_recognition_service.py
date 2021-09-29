#!/usr/bin/env python3
#import roslib; roslib.load_manifest('pick_and_place_application')

import rclpy
from rclpy.node import Node

from pick_and_place_msgs.srv import GetTargetPose

#import rospy
import tf2_ros
import shape_msgs
from geometry_msgs.msg import Pose


#constants
GET_TARGET_POSE_SERVICE = "get_target_pose";

#variables
#ros_node = None #Node('simulation_recognition_node')
#tf_buffer = None
#tf_listener = None


#server callback
def recognition_callback(ros_node, tf_buffer, req, res):
	
	ar_frame = req.ar_tag_frame_id
	world_frame = req.world_frame_id
	shape = req.shape
	
	# lookup tranform
	try:
		trans = tf_buffer.lookup_transform(world_frame,ar_frame, ros_node.get_clock().now())		
		transform = trans.transform
		ros_node.get_logger().info('ar_frame "%s" relative to "%s" detected'%(ar_frame,world_frame))	
		
		#modifying height
		height = shape.dimensions[2] if (len(shape.dimensions)==3) else transform.translation.z

		# creating pose
		pose = Pose()
		pose.position.x = transform.translation.x
		pose.position.y = transform.translation.y
		pose.position.z = height
		pose.orientation.x = transform.rotation.x
		pose.orientation.y = transform.rotation.y
		pose.orientation.z = transform.rotation.z
		pose.orientation.w = transform.rotation.w

		res.target_pose = pose
		res.succeeded = True	
		

	except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
		ros_node.get_logger().error('ar_frame "%s" relative to "%s" not found'%(ar_frame,world_frame))
		res.succeeded = False			
	
	return res

def main(args = None):
	#rospy.init_node("simulation_recognition_node")
	rclpy.init(args = args)
	
	ros_node = Node('fake_recognition_node')

	# creating listener
	tf_buffer = tf2_ros.Buffer()
	tf_listener = tf2_ros.TransformListener(tf_buffer, ros_node)
	service_cb = lambda req, res : recognition_callback(ros_node, tf_buffer, req, res)
	server = ros_node.create_service(GetTargetPose, GET_TARGET_POSE_SERVICE, service_cb)
	
	try:
		rclpy.spin(ros_node)
	except KeyboardInterrupt:
		pass
	
	ros_node.destroy_node()
	rclpy.shutdown()

if __name__ == "__main__":
	main()


		





