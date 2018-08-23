#pragma once
#include <tesseract_planning/trajopt/trajopt_planner.h>
#include <tesseract_ros/kdl/kdl_env.h>

/**
 * @brief The TrajoptPickAndPlaceConstructor class
 */
class TrajoptPickAndPlaceConstructor
{
private:
   /**< @brief Problem Construction Info */
  std::string manipulator_, ee_link_, pick_object_;
  Eigen::Affine3d tcp_;
  tesseract::BasicEnvConstPtr env_;
  tesseract::BasicKinConstPtr kin_;


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

  void addLinearMotion(trajopt::ProblemConstructionInfo& pci, Eigen::Affine3d start_pose, Eigen::Affine3d end_pose, int num_steps, int first_time_step);

  void addInitialJointPosConstraint(trajopt::ProblemConstructionInfo &pci);

  void addJointVelCost(trajopt::ProblemConstructionInfo& pci, double coeff);

  void addCollisionCost(trajopt::ProblemConstructionInfo& pci, double dist_pen, double coeff, int first_step, int last_step);
};
