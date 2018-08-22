#pragma once
#include <tesseract_planning/trajopt/trajopt_planner.h>
#include <tesseract_ros/kdl/kdl_env.h>

/**
 * @brief The TrajoptPickAndPlaceConstructor class
 */
class TrajoptPickAndPlaceConstructor
{
private:
  trajopt::ProblemConstructionInfo pci_; /**< @brief text */
  std::string  manipulator_, ee_link_, pick_object_;
  Eigen::Affine3d tcp_;


public:
  TrajoptPickAndPlaceConstructor(tesseract::BasicEnvConstPtr env, std::string manipulator,
                                 std::string ee_link, std::string pick_object, Eigen::Affine3d tcp = Eigen::Affine3d::Identity());

  trajopt::TrajOptProbPtr generatePickProblem(Eigen::Affine3d& approach_pose, Eigen::Affine3d& final_pose, int steps_per_phase);

  /**
   * @brief t
   *
   * Detailed
   *
   * @param retreat_pose
   * @param approach_pose
   * @param final_pose
   * @param steps_per_phase dfdsfs
   * @return
   */
  trajopt::TrajOptProbPtr generatePlaceProblem(Eigen::Affine3d& retreat_pose, Eigen::Affine3d& approach_pose, Eigen::Affine3d& final_pose,
                                               int steps_per_phase);

  void reset(tesseract::BasicEnvConstPtr env);

  void addLinearMotion(Eigen::Affine3d start_pose, Eigen::Affine3d end_pose, int num_steps, int first_time_step);

  void addInitialJointPosConstraint();

  void addJointVelCost(double coeff);

  void addCollisionCost(double dist_pen, double coeff, int first_step, int last_step);
};
