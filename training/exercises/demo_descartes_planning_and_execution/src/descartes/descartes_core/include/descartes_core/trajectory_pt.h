/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2014, Southwest Research Institute
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
 * trajectory_pt.h
 *
 *  Created on: Jun 5, 2014
 *      Author: Dan Solomon
 */

#ifndef TRAJECTORY_PT_H_
#define TRAJECTORY_PT_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_stl_containers/eigen_stl_vector_container.h>
#include <vector>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include "descartes_core/robot_model.h"
#include "descartes_core/trajectory_pt_transition.h"
#include "descartes_core/trajectory_id.h"


namespace descartes_core
{

/**@brief Frame is a wrapper for an affine frame transform.
 * Frame inverse can also be stored for increased speed in downstream calculations.
 */
struct Frame
{
  Frame(){};
  Frame(const Eigen::Affine3d &a):
    frame(a), frame_inv(a.inverse()) {};

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  Eigen::Affine3d frame;
  Eigen::Affine3d frame_inv;

  static const Frame Identity()
  {
    return Frame(Eigen::Affine3d::Identity());
  }
};


/**@brief A TrajectoryPt is the basis for a Trajectory describing the desired path a robot should execute.
 * The desired robot motion spans both Cartesian and Joint space, and so the TrajectoryPt must have capability
 * to report on both these properties.
 *
 * In practice, an application will create a series of process points,
 * and use these process points to create a Trajectory that can be solved for a robot path.
 * In order to implement this easily, each process point should keep track of the TrajectoryPt id, and
 * provide an interpolation method between points.
 */
DESCARTES_CLASS_FORWARD(TrajectoryPt);
class TrajectoryPt
{
public:
  typedef TrajectoryID ID;
  TrajectoryPt() : id_(TrajectoryID::make_id()) {}
  virtual ~TrajectoryPt() {}

  /**@name Getters for Cartesian pose(s)
   * References to "closest" position are decided by norm of joint-space distance.
   * @{
   */

  /**@brief Get single Cartesian pose associated with closest position of this point to seed_state.
   * (Pose of TOOL point expressed in WOBJ frame).
   * @param seed_state Joint_position seed.
   * @param kinematics Kinematics object used to calculate pose
   * @param pose If successful, affine pose of this state.
   * @return True if calculation successful. pose untouched if return false.
   */
  virtual bool getClosestCartPose(const std::vector<double> &seed_state,
                                  const RobotModel &kinematics, Eigen::Affine3d &pose) const = 0;

  /**@brief Get single Cartesian pose associated with nominal of this point.
    * (Pose of TOOL point expressed in WOBJ frame).
   * @param seed_state Joint_position seed.
   * @param kinematics Kinematics object used to calculate pose
   * @param pose If successful, affine pose of this state.
    * @return True if calculation successful. pose untouched if return false.
    */
  virtual bool getNominalCartPose(const std::vector<double> &seed_state,
                                  const RobotModel &kinematics, Eigen::Affine3d &pose) const = 0;

  /**@brief Get "all" Cartesian poses that satisfy this point.
   * @param kinematics Kinematics object used to calculate pose
   * @param poses Note: Number of poses returned may be subject to discretization used.
   */
  virtual void getCartesianPoses(const RobotModel &kinematics, EigenSTL::vector_Affine3d &poses) const = 0;
  /** @} (end section) */

  /**@name Getters for joint pose(s)
   * References to "closest" position are decided by norm of joint-space distance.
   * @{
   */

  /**@brief Get single Joint pose closest to seed_state.
   * @param seed_state Joint_position seed.
   * @param model Robot mode object used to calculate pose
   * @param joint_pose Solution (if function successful).
   * @return True if calculation successful. joint_pose untouched if return is false.
   */
  virtual bool getClosestJointPose(const std::vector<double> &seed_state,
                                   const RobotModel &model,
                                   std::vector<double> &joint_pose) const = 0;

  /**@brief Get single Joint pose closest to seed_state.
   * @param seed_state Joint_position seed.
   * @param model Robot model object used to calculate pose
   * @param seed_state RobotState used kinematic calculations and joint position seed.
   * @return True if calculation successful. joint_pose untouched if return is false.
   */
  virtual bool getNominalJointPose(const std::vector<double> &seed_state,
                                   const RobotModel &model,
                                   std::vector<double> &joint_pose) const = 0;

  /**@brief Get "all" joint poses that satisfy this point.
   * @param model Robot model  object used to calculate pose
   * @param joint_poses vector of solutions (if function successful). Note: # of solutions may be subject to discretization used.
   */
  virtual void getJointPoses(const RobotModel &model,
                             std::vector<std::vector<double> > &joint_poses) const = 0;
  /** @} (end section) */

  /**@brief Check if state satisfies trajectory point requirements.
   * @param model Robot model  object used to determine validity
   */
  virtual bool isValid(const RobotModel &model) const = 0;

  /**@brief Set discretization. Note: derived classes interpret and use discretization differently.
   * @param discretization Vector of discretization values.
   * @return True if vector is valid length/values.
   */
  virtual bool setDiscretization(const std::vector<double> &discretization) = 0;

  /**@name Getters/Setters for ID
   * @{ */

  /**@brief Get ID associated with this point */
  inline
  ID getID() const
  {
    return id_;
  }

  /**@brief Set ID for this point.
   * @param id Number to set id_ to.
   */
  void setID(const ID &id)
  {
    id_ = id;
  }
  /** @} (end section) */

  /**
   * @brief Makes a copy of the underlying trajectory point and returns a polymorphic handle to it
   * @return A copy, with the same ID, of the underlying point type
   */
  virtual TrajectoryPtPtr copy() const = 0;

  /**
   * @brief Makes a clone of the underlying trajectory point and returns a polymorphic handle to it
   * @return A clone, with the same data but a unique ID, of the underlying point type
   */
  virtual TrajectoryPtPtr clone() const
  {
    TrajectoryPtPtr cp = copy();
    cp->setID(TrajectoryID::make_id());
    return cp;
  }

protected:
  ID                            id_;                    /**<@brief ID associated with this pt. Generally refers back to a process path defined elsewhere. */
  TrajectoryPtTransitionPtr     transition_;            /**<@brief Velocities at, and interpolation method to reach this point **/

};

} /* namespace descartes_core */


#endif /* TRAJECTORY_PT_H_ */
