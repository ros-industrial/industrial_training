#include <test_bed_core/trajopt_pick_and_place_constructor.h>
#include <trajopt/problem_description.hpp>
#include <trajopt_utils/eigen_conversions.hpp>

#include <ros/ros.h>

using namespace trajopt;
using namespace Eigen;

TrajoptPickAndPlaceConstructor::TrajoptPickAndPlaceConstructor(tesseract::BasicEnvConstPtr env,
                                                               std::string manipulator,
                                                               std::string ee_link,
                                                               std::string pick_object,
                                                               Isometry3d tcp)
  : env_(env), manipulator_(manipulator), ee_link_(ee_link), pick_object_(pick_object), tcp_(tcp)
{
  kin_ = env->getManipulator(manipulator_);
}

void TrajoptPickAndPlaceConstructor::addInitialJointPosConstraint(trajopt::ProblemConstructionInfo& pci)
{
  std::shared_ptr<JointConstraintInfo> start_constraint = std::shared_ptr<JointConstraintInfo>(new JointConstraintInfo);

  /* Fill Code:
       . Define the term type (This is a constraint)
       . Define the time step
       . Define the constraint name

  */
  /* ========  ENTER CODE HERE ======== */

  // ENTER CODE HERE:
  // ENTER CODE HERE:
  // ENTER CODE HERE:
  Eigen::VectorXd start_joint_pos = env_->getCurrentJointValues();
  start_constraint->vals = std::vector<double>(start_joint_pos.data(), start_joint_pos.data() + start_joint_pos.rows());
  pci.cnt_infos.push_back(start_constraint);
}

void TrajoptPickAndPlaceConstructor::addJointVelCost(trajopt::ProblemConstructionInfo& pci, double coeff)
{
  std::vector<std::string> joint_names = kin_->getJointNames();
  // Loop over all of the joints
  //  for (std::size_t i = 0; i < joint_names.size(); i++)
  //  {
  std::shared_ptr<JointVelTermInfo> jv(new JointVelTermInfo);
  jv->coeffs = std::vector<double>(1, coeff);
  /* Fill Code:
       . Define the term time (This is a cost)
       . Define the first time step
       . Define the last time step
       . Define the joint name
       . Define the penalty type as sco::squared
  */
  /* ========  ENTER CODE HERE ======== */
  // ENTER CODE HERE:
  // ENTER CODE HERE:
  // ENTER CODE HERE:
  // ENTER CODE HERE:
  // ENTER CODE HERE:
  // ENTER CODE HERE:
  pci.cost_infos.push_back(jv);
  //  }
}

void TrajoptPickAndPlaceConstructor::addJointAccelCost(trajopt::ProblemConstructionInfo& pci, double coeff)
{
  std::vector<std::string> joint_names = kin_->getJointNames();
  // Loop over all of the joints
  for (std::size_t i = 0; i < joint_names.size(); i++)
  {
    std::shared_ptr<JointAccCostInfo> jv(new JointAccCostInfo);
    jv->coeffs = std::vector<double>(1, coeff);
    jv->name = joint_names[i] + "_accel";
    jv->term_type = TT_COST;
    jv->first_step = 0;
    jv->last_step = pci.basic_info.n_steps - 1;
    jv->joint_name = joint_names[i];
    jv->penalty_type = sco::SQUARED;
    pci.cost_infos.push_back(jv);
  }
}

void TrajoptPickAndPlaceConstructor::addTotalTimeCost(ProblemConstructionInfo& pci, double coeff)
{
  std::shared_ptr<TotalTimeTermInfo> time_cost(new TotalTimeTermInfo);
  time_cost->name = "time_cost";
  time_cost->penalty_type = sco::ABS;
  time_cost->weight = coeff;
  time_cost->term_type = TT_COST;
  pci.cost_infos.push_back(time_cost);
}

void TrajoptPickAndPlaceConstructor::addCollisionCost(trajopt::ProblemConstructionInfo& pci,
                                                      double dist_pen,
                                                      double coeff,
                                                      int first_step,
                                                      int last_step)
{
  std::shared_ptr<CollisionCostInfo> collision(new CollisionCostInfo);
  /* Fill Code:
       . Define the cost name
       . Define the term type (This is a cost)
       . Define this cost as continuous
       . Define the first time step
       . Define the last time step
       . Set the cost gap to be 1
       . Define the penalty type as sco::squared
  */
  /* ========  ENTER CODE HERE ======== */
  // ENTER CODE HERE:
  // ENTER CODE HERE:
  // ENTER CODE HERE:
  // ENTER CODE HERE:
  // ENTER CODE HERE:
  // ENTER CODE HERE:
  collision->info = createSafetyMarginDataVector(last_step - first_step + 1, dist_pen, coeff);

  pci.cost_infos.push_back(collision);
}

void TrajoptPickAndPlaceConstructor::addLinearMotion(trajopt::ProblemConstructionInfo& pci,
                                                     Isometry3d start_pose,
                                                     Isometry3d end_pose,
                                                     int num_steps,
                                                     int first_time_step)
{
  // linear delta
  Vector3d xyz_delta = (end_pose.translation() - start_pose.translation()) / (num_steps - 1);

  Quaterniond approach_rotation(start_pose.linear());
  Matrix3d rotation_diff = (start_pose.linear().inverse() * end_pose.linear());
  AngleAxisd aa_rotation_diff(rotation_diff);
  double angle_delta = aa_rotation_diff.angle() / (num_steps - 1);
  Vector3d delta_axis = aa_rotation_diff.axis();

  // Create a series of pose constraints for linear pick motion
  for (int i = 0; i < num_steps; i++)
  {
    /* Fill Code:
         . Create a new shared_ptr<StaticPoseCostInfo>
         . Define the term type (This is a constraint)
         . Set the link constrained as the end effector (see class members)
         . Set the correct time step for this pose
         . Set the pose xyz translation
    */
    /* ========  ENTER CODE HERE ======== */
    // ENTER CODE HERE:
    // ENTER CODE HERE:
    // ENTER CODE HERE:
    // ENTER CODE HERE:
    // ENTER CODE HERE:

    Quaterniond rotation_delta(cos(0.5 * angle_delta * i),
                               delta_axis.x() * sin(0.5 * angle_delta * i),
                               delta_axis.y() * sin(0.5 * angle_delta * i),
                               delta_axis.z() * sin(0.5 * angle_delta * i));
    Quaterniond rotation = rotation_delta * approach_rotation;

    /* Fill Code:
         . Set the pose rotation
         . Set pos_coeffs to all 10s
         . Set rot_coeffs to all 10s
         . Define the pose name as pose_[timestep]
         . pushback the constraint to cnt_infos
    */
    /* ========  ENTER CODE HERE ======== */
    // ENTER CODE HERE:
    // ENTER CODE HERE:
    // ENTER CODE HERE:
    // ENTER CODE HERE:
    // ENTER CODE HERE:
  }
}

TrajOptProbPtr TrajoptPickAndPlaceConstructor::generatePickProblem(Isometry3d& approach_pose,
                                                                   Isometry3d& final_pose,
                                                                   int steps_per_phase)
{
  // create new problem
  trajopt::ProblemConstructionInfo pci(env_);

  /* Fill Code:
   * Define the basic info
       . Set the pci number of steps
       . Set the start_fixed to false
       . Set the manipulator name (see class members)
       . Set dt lower limit
       . Set dt upper limit
  */
  /* ========  ENTER CODE HERE ======== */
  // ENTER CODE HERE:
  // ENTER CODE HERE:
  // ENTER CODE HERE:
  // ENTER CODE HERE:
  // ENTER CODE HERE:

  // Add kinematics
  pci.kin = kin_;

  // Initialize
  // To use STRAIGHT_LINE - a linear interpolation of joint values
//  pci.init_info.type = InitInfo::STRAIGHT_LINE;
//  pci.init_info.data = numericalIK(final_pose);  // Note, take the last value off if has_time=false

  // To use STATIONARY - All steps initialized at start point
  pci.init_info.type = InitInfo::STATIONARY;
  pci.init_info.data = env_->getCurrentJointValues(pci.kin->getName()).replicate(2*steps_per_phase, 1);   // Define the initial guess that is num_steps x num_joints

  // Turn on time parameterization
  pci.init_info.has_time = true;

  /* Fill Code: Define motion
       . Add joint velocity contraint (this->addJointVelCost(pci, 5.0))
       . Add joint acceleration constraint
       . Add total time cost
       . Add initial joint pose constraint
       . Add linear motion contraints from approach_pose to final_pose
  */
  /* ========  ENTER CODE HERE ======== */
  // ENTER CODE HERE:

  // ENTER CODE HERE:

  // ENTER CODE HERE:

  // ENTER CODE HERE:

  // ENTER CODE HERE:

  this->addCollisionCost(pci, 0.025, 20, 0, steps_per_phase);

  TrajOptProbPtr result = ConstructProblem(pci);
  return result;
}

TrajOptProbPtr TrajoptPickAndPlaceConstructor::generatePlaceProblem(Isometry3d& retreat_pose,
                                                                    Isometry3d& approach_pose,
                                                                    Isometry3d& final_pose,
                                                                    int steps_per_phase)
{
  // create new problem
  trajopt::ProblemConstructionInfo pci(env_);

  /* Fill Code: Define the basic info
       . Set the pci number of steps
       . Set the start_fixed to false
       . Set the manipulator name (see class members)
       . Set dt lower limit
       . Set dt upper limit
  */
  /* ========  ENTER CODE HERE ======== */

  // ENTER CODE HERE:
  // ENTER CODE HERE:
  // ENTER CODE HERE:
  // ENTER CODE HERE:
  // ENTER CODE HERE:

  // Add kinematics
  pci.kin = kin_;

  pci.init_info.type = InitInfo::STATIONARY;
  // Define the initial guess that is num_steps x num_joints
  pci.init_info.data = env_->getCurrentJointValues(pci.kin->getName()).replicate(3 * steps_per_phase, 1);
  pci.init_info.has_time = true;

  this->addJointVelCost(pci, 100.0);

  this->addTotalTimeCost(pci, 50.0);

  this->addInitialJointPosConstraint(pci);

  Eigen::Isometry3d start_pose;
  pci.kin->calcFwdKin(start_pose,
                      env_->getState()->transforms.at(kin_->getBaseLinkName()),
                      env_->getCurrentJointValues(),
                      ee_link_,
                      *env_->getState());

  /* Fill Code: Define motion
       . Add linear motion from start_pose to retreat_pose
       . Add linear motion from approach_pose to final_pose
       . Add collision cost
  */
  /* ========  ENTER CODE HERE ======== */
  // ENTER CODE HERE:

  // ENTER CODE HERE:

  // ENTER CODE HERE:

  TrajOptProbPtr result = ConstructProblem(pci);
  return result;
}

Eigen::VectorXd TrajoptPickAndPlaceConstructor::numericalIK(Isometry3d& end_pose)
{
  // Create new problem construction info
  trajopt::ProblemConstructionInfo pci_ik(env_);
  // Only 2 steps
  pci_ik.basic_info.n_steps = 2;
  pci_ik.basic_info.manip = "manipulator";
  pci_ik.basic_info.start_fixed = false;
  pci_ik.init_info.type = InitInfo::STATIONARY;
  pci_ik.kin = kin_;
  pci_ik.init_info.data = env_->getCurrentJointValues(pci_ik.kin->getName());

  // Set initial joint constraint
  this->addInitialJointPosConstraint(pci_ik);

  // Set velocity cost to get "shortest" ik solution
  this->addJointVelCost(pci_ik, 10);

  // Set static pose constraint
  std::shared_ptr<StaticPoseCostInfo> pose_constraint = std::shared_ptr<StaticPoseCostInfo>(new StaticPoseCostInfo);
  pose_constraint->term_type = TT_CNT;
  pose_constraint->link = ee_link_;
  pose_constraint->timestep = 1;

  // Set the position and rotation
  pose_constraint->xyz = end_pose.translation();
  Quaterniond rotation(end_pose.data());
  pose_constraint->wxyz = Vector4d(rotation.w(), rotation.x(), rotation.y(), rotation.z());

  // Set the coefficients and the name
  pose_constraint->pos_coeffs = Vector3d(10.0, 10.0, 10.0);
  pose_constraint->rot_coeffs = Vector3d(10.0, 10.0, 10.0);
  pose_constraint->name = "pose";

  // Add it to the pci
  pci_ik.cnt_infos.push_back(pose_constraint);

  // Construct trajopt problem from info
  TrajOptProbPtr problem = ConstructProblem(pci_ik);

  // Create trust region
  BasicTrustRegionSQP opt(problem);
  opt.initialize(DblVec(problem->GetNumDOF() * 2, 0));
  opt.optimize();

  // Get the joint values for only the last point from the output and return them
  Eigen::VectorXd output = toVectorXd(opt.x());
  return output.segment(problem->GetNumDOF(), problem->GetNumDOF());
}
