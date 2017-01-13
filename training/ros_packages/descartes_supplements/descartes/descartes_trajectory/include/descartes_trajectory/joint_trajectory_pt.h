/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2014, Dan Solomon
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/*
 * joint_trajectory_pt.h
 *
 *  Created on: Oct 3, 2014
 *      Author: Dan Solomon
 */

#ifndef JOINT_TRAJECTORY_PT_H_
#define JOINT_TRAJECTORY_PT_H_

#include <vector>
#include "descartes_core/trajectory_pt.h"

namespace descartes_trajectory
{
/**
 * @brief Structure to specify a valid joint range [lower, upper] with a nominal position given by 'nominal'.
 */
struct TolerancedJointValue
{
  TolerancedJointValue(double nominal, double lower, double upper) : nominal(nominal), lower(lower), upper(upper)
  {
  }

  TolerancedJointValue(double nominal) : nominal(nominal), lower(nominal), upper(nominal)
  {
  }

  double range() const
  {
    return upper - lower;
  }

  double nominal, lower, upper;
};

/**@brief Joint Trajectory Point used to describe a joint goal for a robot trajectory.
 *
 * Background:
 * The TOOL is something held by the robot. It is located relative to robot wrist/tool plate.
 * The WOBJ is something that exists in the world/global environment that is not held by robot.
 *
 * For a JointTrajectoryPt, the transform from wrist to tool, and base to workobject, are defined by fixed frames.
 * These transforms are important when calculating interpolation.
 * The joint position is specified as a nominal with upper/lower tolerances.
 *
 * The get*Pose() methods of JointTrajectoryPt try to set joint positions of a robot such that @e tool_ is coincident
 *with @e wobj_.
 */
class JointTrajectoryPt : public descartes_core::TrajectoryPt
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;  // TODO is this needed when Frame already has it?
public:
  /**
    @brief All frames initialized to Identity, joint
    values left empty
    */
  JointTrajectoryPt(const descartes_core::TimingConstraint &timing = descartes_core::TimingConstraint());

  /**
    @brief Full joint trajectory point constructor
    @param joints Fixed joint position with tolerance
    @param tool Transform from robot wrist to active tool pt.
    @param wobj Transform from world to active workobject pt.
    */
  JointTrajectoryPt(const std::vector<TolerancedJointValue> &joints, const descartes_core::Frame &tool,
                    const descartes_core::Frame &wobj,
                    const descartes_core::TimingConstraint &timing = descartes_core::TimingConstraint());

  /**
    @brief Full joint trajectory point constructor
    @param joints Fixed joint position with tolerance
    */
  JointTrajectoryPt(const std::vector<TolerancedJointValue> &joints,
                    const descartes_core::TimingConstraint &timing = descartes_core::TimingConstraint());

  /**
    @brief Full joint trajectory point constructor
    @param joints Fixed joint position
    */
  JointTrajectoryPt(const std::vector<double> &joints,
                    const descartes_core::TimingConstraint &timing = descartes_core::TimingConstraint());

  virtual ~JointTrajectoryPt(){};

  /**@name Getters for Cartesian pose(s)
   * @{
   */

  // TODO complete
  virtual bool getClosestCartPose(const std::vector<double> &seed_state, const descartes_core::RobotModel &model,
                                  Eigen::Affine3d &pose) const;

  // TODO complete
  virtual bool getNominalCartPose(const std::vector<double> &seed_state, const descartes_core::RobotModel &model,
                                  Eigen::Affine3d &pose) const;

  // TODO complete
  virtual void getCartesianPoses(const descartes_core::RobotModel &model, EigenSTL::vector_Affine3d &poses) const;
  /** @} (end section) */

  /**@name Getters for joint pose(s)
   * @{
   */

  // TODO complete
  virtual bool getClosestJointPose(const std::vector<double> &seed_state, const descartes_core::RobotModel &model,
                                   std::vector<double> &joint_pose) const;
  // TODO complete
  virtual bool getNominalJointPose(const std::vector<double> &seed_state, const descartes_core::RobotModel &model,
                                   std::vector<double> &joint_pose) const;

  // TODO complete
  virtual void getJointPoses(const descartes_core::RobotModel &model,
                             std::vector<std::vector<double> > &joint_poses) const;
  /** @} (end section) */

  // TODO complete
  virtual bool isValid(const descartes_core::RobotModel &model) const;

  // TODO complete
  /**@brief Set discretization. Each joint can have a different discretization.
   * @param discretization Vector of discretization values. If length=1, set all elements of discretization_ are set to
   * value.
   * @return True if vector is length 1 or length(joint_position_) and value[ii] are within 0-range(joint_position[ii]).
   */
  virtual bool setDiscretization(const std::vector<double> &discretization);

  virtual descartes_core::TrajectoryPtPtr copy() const
  {
    return descartes_core::TrajectoryPtPtr(new JointTrajectoryPt(*this));
  }

  void setJoints(const std::vector<TolerancedJointValue> &joints);

  inline void setTool(const descartes_core::Frame &tool)
  {
    tool_ = tool;
  }

  inline void setWobj(const descartes_core::Frame &wobj)
  {
    wobj_ = wobj;
  }
  /**@} (end Setters section) */

  inline const std::vector<double> &nominal() const
  {
    return nominal_;
  }

  inline const std::vector<double> &upper() const
  {
    return upper_;
  }

  inline const std::vector<double> &lower() const
  {
    return lower_;
  }

protected:
  std::vector<double> nominal_;
  std::vector<double> lower_;
  std::vector<double> upper_;
  std::vector<double> discretization_; /**<@brief How finely to discretize each joint */

  /** @name JointTrajectoryPt transforms. Used in get*CartPose() methods and for interpolation.
   *  @{
   */
  descartes_core::Frame tool_; /**<@brief Transform from robot wrist to active tool pt. */
  descartes_core::Frame wobj_; /**<@brief Transform from world to active workobject pt. */
  /** @} (end section) */
};

} /* namespace descartes_trajectory */

#endif /* JOINT_TRAJECTORY_PT_H_ */
