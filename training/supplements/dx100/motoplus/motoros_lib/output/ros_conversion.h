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

#ifndef ROS_CONVERSION_H
#define ROS_CONVERSION_H

#ifdef ROS
#include "simple_message/joint_data.h"
#endif

#ifdef MOTOPLUS
#include "joint_data.h"
#endif

namespace motoman
{
namespace ros_conversion
{

/**
 * \brief Enumeration of motoman joint indicies (as returned by the getPos
 * library calls.
 */
namespace MotomanJointIndexes
{
enum MotomanJointIndex
{
  S = 0, 
  L = 1, 
  U = 2,
  R = 3,
  B = 4,
  T = 5,
  E = 6,
  COUNT = E
};
}
typedef MotomanJointIndexes::MotomanJointIndex MotomanJointIndex;

/**
 * \brief Enumeration of ROS joint indicies.  In reality these are not
 * fixed.  The ROS message structure maps joints by name and theoretically
 * would allow for any order.  Generally the order from base to tip is 
 * maintained (This is what the enumeration assumes).
 */
namespace RosJointIndexes
{
enum RosJointIndex
{
  S = 0, 
  L = 1, 
  E = 2,  
  U = 3,
  R = 4,
  B = 5,
  T = 6,
  COUNT = T
};
}
typedef RosJointIndexes::RosJointIndex RosJointIndex;

/**
 * \brief Converts a motoplus joint (radians, in motoplus order) to
 * a ros joint (radians, in ros order, JointData format)
 *
 * \param motoplus joints to convert
 * \param ros joint (returned)
 */
void toRosJoint(float* mp_joints, 
				industrial::joint_data::JointData & ros_joints);

/**
 * \brief Converts a ros joint (in ros order, JointData format) to
 * a motoplus joint (in motoplus order)
 *
 * \param ros joint to convert
 * \param motoplus joints (returned)
 */
void toMpJoint(industrial::joint_data::JointData & ros_joints,
				float* mp_joints);

// These functions change the JointData in place which has lead to some dangerous
// bugs when they are called more than once.  Use the functions above
// that swap joint order and perform unit conversion automatically
//DEPRECATED
void toRosJointOrder(industrial::joint_data::JointData & joints);
//DEPRECATED
void toMotomanJointOrder(industrial::joint_data::JointData & joints);
    
    
} //ros_conversion
} //motoman


#endif //ROS_CONVERSION_H
