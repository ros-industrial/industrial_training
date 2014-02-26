#!/usr/bin/env python
import argparse
import shlex
import roslib; roslib.load_manifest('collision_avoidance_pick_and_place')
import rospy
import tf
import shape_msgs
import moveit_msgs
import std_msgs.msg
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


#constants
COLLISION_OBJECT_TOPIC = "collision_objects";

def create_collision_object(shape_type,pos,size,frame_id,op,object_id):

	col = CollisionObject()
	col.id = object_id
	col.operation = op
	col.header.frame_id=frame_id
	
	# create primitive
	primitive = SolidPrimitive()
	primitive.type = shape_type
	primitive.dimensions = size

	# create pose
	pose = Pose()
	pose.position.x = pos[0]
	pose.position.y = pos[1]
	pose.position.z = pos[2]
	pose.orientation.x = pose.orientation.y = pose.orientation.z = 0
	pose.orientation.w = 1
	

	col.primitives = [primitive]
	col.primitive_poses = [pose]
	
	return col



if __name__ == "__main__":


	rospy.init_node("collision_object_publisher")

	# create publisher
	collision_publisher = rospy.Publisher('collision_objects',CollisionObject)

	# argument parser
	parser=argparse.ArgumentParser(description="Collision Object description")

	# adding arguments
	parser.add_argument("-t","--type",type=int,help="primitive type: BOX=1, SPHERE=2, CYLINDER=3, CONE=4",choices=[1,2,3,4])
	parser.add_argument("-s","--size",type=float, metavar='N',nargs=3,help="size: [l w h]")
	parser.add_argument("-p","--position",type=float,metavar='N',nargs=3,help="position: [x y z]")
	parser.add_argument("-f","--frameid",type=str,help="frame id")
	parser.add_argument("-o","--operation",type=int,help="operations: ADD=0, DELETE=1 ",choices=[0,1])
	parser.add_argument("-i","--id",type=str,help="object id")	
	parser.add_argument("-q","--quit",action='store_true',help="quit script")

	args = parser.parse_args()
	
	rospy.loginfo("collision object publisher ready")
	while True:
		arg_string=raw_input('Enter object: ')
		args = parser.parse_args(shlex.split(arg_string))
		if not args.quit:
			print "type: %s , size: [%s], position: [%s], frameid: %s" %(args.type,','.join(map(str,args.size)),
				','.join(map(str,args.position)),args.frameid)
			msg = create_collision_object(args.type,args.position,args.size,args.frameid,args.operation,args.id)
			collision_publisher.publish(msg)
		else:
			break

		





