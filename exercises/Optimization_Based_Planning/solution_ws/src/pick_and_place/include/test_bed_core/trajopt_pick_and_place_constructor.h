#pragma once
#include <tesseract_planning/trajopt/trajopt_planner.h>
#include <tesseract_ros/kdl/kdl_env.h>
#include <trajopt/problem_description.hpp>


/**
 * @brief The TrajoptPickAndPlaceConstructor class
 */
class TrajoptPickAndPlaceConstructor
{
private:
  /**< @brief Problem Construction Info */
  std::string manipulator_, ee_link_, pick_object_;
  Eigen::Isometry3d tcp_;             /**< @brief Tool center point offset */
  tesseract::BasicEnvConstPtr env_; /**< @brief Environment description */
  tesseract::BasicKinConstPtr kin_; /**< @brief Kinematics description */

public:
  TrajoptPickAndPlaceConstructor(tesseract::BasicEnvConstPtr env,
                                 std::string manipulator,
                                 std::string ee_link,
                                 std::string pick_object,
                                 Eigen::Isometry3d tcp = Eigen::Isometry3d::Identity());

  /**
   * @brief Constrains initial joint positiions
   * @param pci - The trajopt problem construction info to which the cost is added
   */
  void addInitialJointPosConstraint(trajopt::ProblemConstructionInfo& pci);

  /**
   * @brief Adds a cost to the optimization of the joint velocities
   * @param pci - The trajopt problem construction info to which the cost is added
   * @param coeff -
   */
  void addJointVelCost(trajopt::ProblemConstructionInfo& pci, double coeff);

  /**
   * @brief Adds a collision cost to a subset of the time steps in a trajopt problem construction info
   * @param pci - The trajopt problem construction info to which the cost is added
   * @param dist_pen
   * @param coeff
   * @param first_step - First step to which the collision cost is added
   * @param last_step
   */
  void
  addCollisionCost(trajopt::ProblemConstructionInfo& pci, double dist_pen, double coeff, int first_step, int last_step);
  /**
   * @brief Adds a linear move to the problem construction info
   * @param pci - The trajopt problem construction info to which the move is added
   * @param start_pose - The starting pose of the linear move
   * @param end_pose - The end pose of the linear move
   * @param num_steps -  Number of steps for the move.
   * @param first_time_step - Time step at which the move is added
   */
  void addLinearMotion(trajopt::ProblemConstructionInfo& pci,
                       Eigen::Isometry3d start_pose,
                       Eigen::Isometry3d end_pose,
                       int num_steps,
                       int first_time_step);

  /**
   * @brief Generates a trajopt problem for a "pick" move
   * Consists of 2 phases - a free space move to approach_pose and a linear move to final_pose
   * @param approach_pose - Pose moved to prior to picking
   * @param final_pose - Pose moved to for the pick operation
   * @param steps_per_phase - Number of steps per phase. Total move is steps_per_phase*2
   * @return
   */
  trajopt::TrajOptProbPtr generatePickProblem(Eigen::Isometry3d& approach_pose,
                                              Eigen::Isometry3d& final_pose,
                                              int steps_per_phase);

  /**
   * @brief Generates a trajopt problem for a "place" move
   * Consists of 3 phases - linearly lifting the object to retreat_pose, a free space move to approach_pose, and then
   * linearly placing the object to final_pose
   * @param retreat_pose - Pose to which the object is moved from it's starting pose
   * @param approach_pose  - Pose to which the object is moved prior to placing
   * @param final_pose - Final "placed" pose
   * @param steps_per_phase - Number of steps per move phase. Total move length is steps_per_phase*3
   * @return
   */
  trajopt::TrajOptProbPtr generatePlaceProblem(Eigen::Isometry3d& retreat_pose,
                                               Eigen::Isometry3d& approach_pose,
                                               Eigen::Isometry3d& final_pose,
                                               int steps_per_phase);
};
