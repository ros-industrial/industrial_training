/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 	* Redistributions of source code must retain the above copyright
 * 	notice, this list of conditions and the following disclaimer.
 * 	* Redistributions in binary form must reproduce the above copyright
 * 	notice, this list of conditions and the following disclaimer in the
 * 	documentation and/or other materials provided with the distribution.
 * 	* Neither the name of the Southwest Research Institute, nor the names
 *	of its contributors may be used to endorse or promote products derived
 *	from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
 
#ifdef ROS
#include "dx100/ros_conversion.h"
#include "simple_message/log_wrapper.h"
using std::min;           // MotoPlus provides min(), but not std::min()
#define MAX_PULSE_AXES 8  // since MotoPlus.h is not availble in ROS
#endif

#ifdef MOTOPLUS
#include "ros_conversion.h"
#include "log_wrapper.h"
#endif

using namespace industrial::joint_data;

namespace motoman
{

namespace ros_conversion
{

void toRosJoint(float* mp_joints, JointData & ros_joints)
{
    // convert from float-array to JointData
    JointData temp;
    int minJoints = min(MAX_PULSE_AXES, ros_joints.getMaxNumJoints());
    for (int i=0; i<minJoints; ++i)
        temp.setJoint(i, mp_joints[i]);
        
    // re-order joints to ROS ordering (for known joint-pairs)
    ros_joints.copyFrom(temp);  // initialize to motoman ordering
    ros_joints.setJoint(RosJointIndexes::S, temp.getJoint(MotomanJointIndexes::S));
    ros_joints.setJoint(RosJointIndexes::L, temp.getJoint(MotomanJointIndexes::L));
    ros_joints.setJoint(RosJointIndexes::U, temp.getJoint(MotomanJointIndexes::U));
    ros_joints.setJoint(RosJointIndexes::R, temp.getJoint(MotomanJointIndexes::R));
    ros_joints.setJoint(RosJointIndexes::B, temp.getJoint(MotomanJointIndexes::B));
    ros_joints.setJoint(RosJointIndexes::T, temp.getJoint(MotomanJointIndexes::T));
    ros_joints.setJoint(RosJointIndexes::E, temp.getJoint(MotomanJointIndexes::E));
}

void toMpJoint(industrial::joint_data::JointData & ros_joints,
				float* mp_joints)
{
    // re-order joints to MotoPlus ordering (for known joint-pairs)
    JointData temp;
    temp.copyFrom(ros_joints);
    temp.setJoint(MotomanJointIndexes::S, ros_joints.getJoint(RosJointIndexes::S));
    temp.setJoint(MotomanJointIndexes::L, ros_joints.getJoint(RosJointIndexes::L));
    temp.setJoint(MotomanJointIndexes::U, ros_joints.getJoint(RosJointIndexes::U));
    temp.setJoint(MotomanJointIndexes::R, ros_joints.getJoint(RosJointIndexes::R));
    temp.setJoint(MotomanJointIndexes::B, ros_joints.getJoint(RosJointIndexes::B));
    temp.setJoint(MotomanJointIndexes::T, ros_joints.getJoint(RosJointIndexes::T));
    temp.setJoint(MotomanJointIndexes::E, ros_joints.getJoint(RosJointIndexes::E));

    // convert from JointData to float-array
    memset(mp_joints, 0, MAX_PULSE_AXES*sizeof(float));  // initialize to zero
    int minJoints = min(MAX_PULSE_AXES, ros_joints.getMaxNumJoints());
    for (int i=0; i<minJoints; ++i)
        mp_joints[i] = temp.getJoint(i);
}

			
void toRosJointOrder(JointData & joints)
{
    //LOG_DEBUG("Swapping to ROS joint order");
    JointData swap;
    swap.setJoint(RosJointIndexes::S, joints.getJoint(MotomanJointIndexes::S));
    swap.setJoint(RosJointIndexes::L, joints.getJoint(MotomanJointIndexes::L));
    swap.setJoint(RosJointIndexes::U, joints.getJoint(MotomanJointIndexes::U));
    swap.setJoint(RosJointIndexes::R, joints.getJoint(MotomanJointIndexes::R));
    swap.setJoint(RosJointIndexes::B, joints.getJoint(MotomanJointIndexes::B));
    swap.setJoint(RosJointIndexes::T, joints.getJoint(MotomanJointIndexes::T));
    swap.setJoint(RosJointIndexes::E, joints.getJoint(MotomanJointIndexes::E));
    joints.copyFrom(swap);
}

void toMotomanJointOrder(JointData & joints)
{
    //LOG_DEBUG("Swapping to motoman joint order");
    JointData swap;
    swap.setJoint(MotomanJointIndexes::S, joints.getJoint(RosJointIndexes::S));
    swap.setJoint(MotomanJointIndexes::L, joints.getJoint(RosJointIndexes::L));
    swap.setJoint(MotomanJointIndexes::U, joints.getJoint(RosJointIndexes::U));
    swap.setJoint(MotomanJointIndexes::R, joints.getJoint(RosJointIndexes::R));
    swap.setJoint(MotomanJointIndexes::B, joints.getJoint(RosJointIndexes::B));
    swap.setJoint(MotomanJointIndexes::T, joints.getJoint(RosJointIndexes::T));
    swap.setJoint(MotomanJointIndexes::E, joints.getJoint(RosJointIndexes::E));
    joints.copyFrom(swap);
}

} //ros_conversion
} //motoman
