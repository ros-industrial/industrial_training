#!/usr/bin/env python
import roslib; roslib.load_manifest('pick_and_place_exercise')
import rospy
import tf
import shape_msgs
import pick_and_place_exercise.srv
from pick_and_place_exercise.srv import *


#constants
TARGET_RECOGNITION_SERVICE = "target_recognition";

#variables
tf_listener = None


#server callback
def recognition_callback(req):

	
	ar_frame = req.ar_tag_frame_id
	world_frame = req.world_frame_id
	shape = req.shape


	# response object
	res = GetTargetPoseResponse()
	
	# lookup tranform
	try:
		(trans,rot) = tf_listener.lookupTransform(world_frame,ar_frame,rospy.Time(0))
		rospy.loginfo("ar_frame '%s' relative to '%s' detected",ar_frame,world_frame)
		
		#modifying height
		trans.z = shape.dimensions[2]

		# creating pose
		pose = geometry_msgs.msg.Pose()
		pose.position.x = trans.x
		pose.position.y = trans.y
		pose.position.z = trans.z
		pose.orientation.x = rot.x
		pose.orientation.y = rot.y
		pose.orientation.z = rot.z
		pose.orientation.w = rot.w

		res.target_pose = pose
		res.succeeded = True	
		

	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		rospy.logerr("ar_frame '%s' relative to '%s' not found",ar_frame,world_frame)
		res.succeeded = False	
		
	
	return res

if __name__ == "__main__":


	rospy.init_node("simulation_recognition_node")

	# creating listener
	tf_listener = tf.TransformListener()

	# creating server
	server = rospy.Service(TARGET_RECOGNITION_SERVICE,GetTargetPose,recognition_callback)
	
	rospy.loginfo("recognition service server ready")
	while not rospy.is_shutdown():
	
		rospy.sleep(0.2)
		





