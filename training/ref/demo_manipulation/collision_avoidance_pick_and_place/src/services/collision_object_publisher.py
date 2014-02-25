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


#constants
COLLISION_OBJECT_TOPIC = "collision_objects";

#command line arguments
ARG_SHAPE_TYPE='-t'
ARG_SIZE = '-s'
ARG_POSITION= '-p'
ARG_FRAME_ID='-f'



#variables
tf_listener = None	

# shape creation functions

def create_collision_object(shape_type,pos,size,frame_id,op,object_id):

	col = CollisionObject
	col.id = object_id
	col.type = shape_type
	col.operation = op
	#print col.header
	#col.header.frame_id=frame_id
	return col.header



if __name__ == "__main__":


	rospy.init_node("collision_object_publisher")

	# create publisher
	collision_publisher = rospy.Publisher('collision_objects',CollisionObject)

	# argument parser
	parser=argparse.ArgumentParser(description="Collision Object description")

	# adding arguments
	parser.add_argument("-t","--type",type=int,help="shape type",choices=[0,1,2])
	parser.add_argument("-s","--size",type=float, metavar='N',nargs=3,help="size [l w h]")
	parser.add_argument("-p","--position",type=float,metavar='N',nargs=3,help="position [x y z]")
	parser.add_argument("-f","--frameid",type=str,help="frame id")
	parser.add_argument("-o","--operation",type=int,help="operations[ADD=0,DELETE=1]",choices=[0,1])
	parser.add_argument("-i","--id",type=str,help="object id")
	
	parser.add_argument("-q","--quit",action='store_true',help="quit script")
	
	rospy.loginfo("collision object publisher ready")
	while True:
		arg_string=raw_input('-->')
		args = parser.parse_args(shlex.split(arg_string))
		if not args.quit:
			print "type: %s , size: [%s], position: [%s], frameid: %s" %(args.type,','.join(map(str,args.size)),
				','.join(map(str,args.position)),args.frameid)
			msg = create_collision_object(args.type,args.position,args.size,args.frameid,args.operation,args.id)
			collision_publisher.publish(msg)
		else:
			break

		





