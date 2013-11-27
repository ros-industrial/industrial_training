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

#include "dx100/utils.h"
#include "dx100/definitions.h"
#include "dx100/trajectory_job.h"
#include "simple_message/joint_traj.h"
#include "simple_message/log_wrapper.h"

#include <vector>
#include <iostream>
#include <fstream>
#include <gtest/gtest.h>

using namespace motoman::utils;
using namespace motoman::trajectory_job;
using namespace industrial::joint_traj;
using namespace industrial::joint_traj_pt;
using namespace industrial::joint_data;


TEST(Utils, hasSuffix)
{
  std::string test = "prefix_root_suffix";
  std::string suffix = "suffix";
  std::string root = "root";
  std::string prefix = "prefix";
  std::string empty = "";
  EXPECT_TRUE(hasSuffix(test, test));
  EXPECT_TRUE(hasSuffix(test, suffix));
  EXPECT_FALSE(hasSuffix(test, root));
  EXPECT_FALSE(hasSuffix(test, prefix));
  EXPECT_TRUE(hasSuffix(test, empty));

}

TEST(Utils, checkJointNames)
{
	trajectory_msgs::JointTrajectoryPtr base_msg(new trajectory_msgs::JointTrajectory());
	trajectory_msgs::JointTrajectoryConstPtr msg(base_msg);
  EXPECT_TRUE(checkJointNames(msg));
  base_msg->joint_names.push_back("joint_s");
  EXPECT_TRUE(checkJointNames(msg));
  base_msg->joint_names.push_back("prefix_joint_l");
  EXPECT_TRUE(checkJointNames(msg));
  base_msg->joint_names.push_back("bad_joint");
  EXPECT_FALSE(checkJointNames(msg));
  base_msg->joint_names.clear();
  base_msg->joint_names.resize(motoman::parameters::Parameters::JOINT_SUFFIXES_SIZE + 1 );
  EXPECT_FALSE(checkJointNames(msg));

  base_msg->joint_names.clear();
  for (int i = 0; i < motoman::parameters::Parameters::JOINT_SUFFIXES_SIZE; i++)
  {
  	base_msg->joint_names.push_back(motoman::parameters::Parameters::JOINT_SUFFIXES[i]);
  }
  EXPECT_TRUE(checkJointNames(msg));
}

TEST(Utils, toMotomanVelocity)
{
  std::vector<double> vel;
  std::vector<double> lim;

  vel.push_back(1.0);
  lim.push_back(2.0);

  EXPECT_FLOAT_EQ(toMotomanVelocity(lim, vel), 1.0/2.0);
  EXPECT_FLOAT_EQ(toMotomanVelocity(vel, lim), 1.0);

  vel.push_back(2.0);
  lim.push_back(3.0);

  EXPECT_FLOAT_EQ(toMotomanVelocity(lim, vel), 2.0/3.0);

  vel.push_back(3.0);
  EXPECT_FLOAT_EQ(toMotomanVelocity(lim, vel), 0.0);

  lim.push_back(4.0);
  lim.push_back(5.0);
  EXPECT_FLOAT_EQ(toMotomanVelocity(lim, vel), 0.0);

}

TEST(DISABLED_TrajectoryJob, init)
{

  using namespace std;

  TrajectoryJob job;
  JointTraj traj;
  JointTrajPt pt;
  JointData init;
  string job_name = "JOB_NAME";
  string job_ext = ".JBI";

  const int TRAJ_SIZE = 50;
  const int BIG_JOB_BUFFER_SIZE = 500000;
  char bigJobBuffer[BIG_JOB_BUFFER_SIZE];
  const int SMALL_JOB_BUFFER_SIZE = 10;
  char smallJobBuffer[SMALL_JOB_BUFFER_SIZE];

  for (int i = 1; i <= TRAJ_SIZE; i++)
  {
    //ROS_DEBUG("Initializing joint: %d", i);
    for (int j = 0; j < init.getMaxNumJoints(); j++)
    {
      //ROS_DEBUG("Initializing point: %d.%d", i, j);
      ASSERT_TRUE(init.setJoint(j, (i*(j+1))));
    }
    pt.init(i, init, 11.1111, 0.01);
    ASSERT_TRUE(traj.addPoint(pt));
  }

  EXPECT_FALSE(job.init("this name is way too long to be the name of a file and this should fail"));
  ASSERT_TRUE(job.init((char*)job_name.c_str()));

  EXPECT_FALSE(job.toJobString(traj, &smallJobBuffer[0], SMALL_JOB_BUFFER_SIZE));
  // Test is disabled because the line below fails due to missing conversions functions.  This
  // ability has been moved to the controller and now is dynamically updated at runtime.  There
  // may not be a solution to this issue.
  EXPECT_TRUE(job.toJobString(traj, &bigJobBuffer[0], BIG_JOB_BUFFER_SIZE));
  ofstream file;
  job_name.append(job_ext);
  file.open(job_name.c_str());
  ASSERT_TRUE(file.is_open());
  file << bigJobBuffer;
  file.close();


}



// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

