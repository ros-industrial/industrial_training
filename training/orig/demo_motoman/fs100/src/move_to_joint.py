#!/usr/bin/python

import sys, yaml
import roslib; roslib.load_manifest('fs100')
import rospy, rosbag
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# display command usage
def print_usage():
    print '\nUsage:\n\n  move_to_joint.py endPos [duration]'
    print '     where endPos is: "[J1, J2, J3,...]" in radians'
    print '     and duration (OPTIONAL) is the expected move duration (in seconds)'
    print '\n  move_to_joint.py trajectory.bag [duration]'
    print '     will move to the first JointTrajectory point in a bag file'
    print '     where duration (OPTIONAL) is the expected move duration (in seconds)\n'

# build a simple trajectory from the start to end position
#   - for the FS100, we can get by with a simple 2-point trajectory
#   - the controller handles the necessary accel/decel to make smooth motion
def build_traj(start, end, duration):

  if sorted(start.name) <> sorted(end.name):
    rospy.logerr('Start and End position joint_names mismatch')
    raise

  # assume the start-position joint-ordering
  joint_names = start.name
 
  start_pt = JointTrajectoryPoint()
  start_pt.positions = start.position
  start_pt.velocities = [0]*len(start.position)
  start_pt.time_from_start = rospy.Duration(0.0)

  end_pt = JointTrajectoryPoint()
  for j in joint_names:
    idx = end.name.index(j)
    end_pt.positions.append(end.position[idx])  # reorder to match start-pos joint ordering
    end_pt.velocities.append(0)
  end_pt.time_from_start = rospy.Duration(duration)

  return JointTrajectory(joint_names=joint_names, points=[start_pt, end_pt])
  
# read the current robot position from the "joint_states" topic
def get_cur_pos():
  try:
    return rospy.wait_for_message("joint_states", JointState, 5.0)
  except (rospy.ROSException, rospy.ROSInterruptException):
    rospy.logerr('Unable to read current position')
    raise

# wait for subscribers
def wait_for_subs(pub, num_subs, min_time, timeout):
  end = rospy.Time.now() + rospy.Duration(timeout)
  rospy.sleep(min_time)

  r = rospy.Rate(10)  # check at 10Hz
  while (pub.get_num_connections() < num_subs) and (rospy.Time.now() < end) and not rospy.is_shutdown():
    r.sleep()

  return (pub.get_num_connections() >= num_subs)

# move the robot to the specified position
#   - read the current robot position
#   - generate a trajectory from the current position to target position
#   - publish the trajectory command
def move_to_joint(end_pos, duration):

  traj = build_traj(get_cur_pos(), end_pos, duration)

  # wait for subscribers to connect
  pub = rospy.Publisher('joint_path_command', JointTrajectory)
  if not wait_for_subs(pub, 1, 0.5, 2.0):
    rospy.logwarn('Timeout while waiting for subscribers.  Publishing trajectory anyway.')

  pub.publish(traj)

# extract the first JointTrajectory point from a bag file
#   - if multiple messages, use the first message on the specified topic
def get_pos_from_bag(bag_file, topic_name):
  with rosbag.Bag(bag_file) as bag:
    msgs = list(bag.read_messages(topic_name))

  if len(msgs) <> 1:
    rospy.logwarn("Multiple trajectories found.  Exporting first trajectory only")

  traj = msgs[0][1]

  return JointState(name=traj.joint_names, position=traj.points[0].positions)

# get the typical list of motoman joint-names, based on DOF-count
#   - override default list with ROS param '~joint_names', if present
def get_joint_names(num_joints):

  if num_joints == 6:
    default_names = ['joint_'+j for j in ['s','l','u','r','b','t']]
  elif num_joints == 7:
    default_names = ['joint_'+j for j in ['s','l','e','u','r','b','t']]
  else:
    default_names = ''

  return rospy.get_param('~joint_names', default_names)

# parse the input arguments
def parse_args(args):
  if len(args) < 1 or len(args) > 2:
    print_usage()
    raise ValueError("Illegal number of arguments")

  # check if first argument is bag-file, otherwise assume array of joint-positions
  try:
    end_pos = get_pos_from_bag(args[0], 'joint_path_command')
  except:
    pos = yaml.load(args[0])
    names = get_joint_names(len(pos))
    end_pos = JointState(name=names, position=pos)

  duration = float(args[1]) if len(args)>1 else 10.0

  return (end_pos, duration)

def main(argv):
  rospy.init_node('move_to_joint')

  try:
    (end_pos, duration) = parse_args(argv)
    move_to_joint(end_pos, duration)
  except:
    rospy.logerr('Unable to move to commanded position. Aborting.')
    raise  # allow default python exception-handler to print exception trace

if __name__ == "__main__":
  main(sys.argv[1:])

