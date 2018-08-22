#include <test_bed_core/trajopt_pick_and_place_constructor.h>
#include <trajopt/problem_description.hpp>

#include <ros/ros.h>


using namespace trajopt;
using namespace Eigen;

TrajoptPickAndPlaceConstructor::TrajoptPickAndPlaceConstructor(tesseract::BasicEnvConstPtr env,
                                                               std::string manipulator, std::string ee_link,
                                                               std::string pick_object, Affine3d tcp)
  : pci_(env), manipulator_(manipulator), ee_link_(ee_link), pick_object_(pick_object), tcp_(tcp)
{
}

TrajOptProbPtr TrajoptPickAndPlaceConstructor::generatePickProblem(Affine3d& approach_pose, Affine3d& final_pose, int steps_per_phase)
{
  reset(pci_.env);

  // basic info
  pci_.basic_info.n_steps = steps_per_phase * 2;
  pci_.basic_info.start_fixed = false;
  pci_.basic_info.manip = manipulator_;

  // kinematics
  pci_.kin = pci_.env->getManipulator(manipulator_);

  // TODO use joint interpolated or seed from other planner?

  pci_.init_info.type = InitInfo::STATIONARY;
  pci_.init_info.data = pci_.env->getCurrentJointValues(pci_.kin->getName());


  this->addJointVelCost(5.0);

  this->addInitialJointPosConstraint();

  this->addLinearMotion(approach_pose, final_pose, steps_per_phase, steps_per_phase);

  this->addCollisionCost(0.025, 20, 0, steps_per_phase);

  TrajOptProbPtr result = ConstructProblem(pci_);
  return result;
}

TrajOptProbPtr TrajoptPickAndPlaceConstructor::generatePlaceProblem(Affine3d& retreat_pose, Affine3d& approach_pose,
                                                                     Affine3d& final_pose, int steps_per_phase)
{
  reset(pci_.env);

  // basic info
  pci_.basic_info.n_steps = steps_per_phase * 3;
  pci_.basic_info.start_fixed = false;
  pci_.basic_info.manip = manipulator_;

  // kinematics
  pci_.kin = pci_.env->getManipulator(manipulator_);

  // TODO use joint interpolated or seed from other planner?
  pci_.init_info.type = InitInfo::STATIONARY;
  pci_.init_info.data = pci_.env->getCurrentJointValues(pci_.kin->getName());

  this->addJointVelCost(5.0);

  this->addInitialJointPosConstraint();

  Eigen::Affine3d start_pose;
  pci_.kin->calcFwdKin(start_pose, pci_.env->getState()->transforms.at(pci_.kin->getBaseLinkName()), pci_.env->getCurrentJointValues(),
                       ee_link_, *pci_.env->getState());

  this->addLinearMotion(start_pose, retreat_pose, steps_per_phase, 0);

  this->addLinearMotion(approach_pose, final_pose, steps_per_phase, steps_per_phase * 2);

  this->addCollisionCost(0.025, 20, steps_per_phase, steps_per_phase * 2 - 1);

  TrajOptProbPtr result = ConstructProblem(pci_);
  return result;
}

void TrajoptPickAndPlaceConstructor::reset(tesseract::BasicEnvConstPtr env)
{
  pci_ = ProblemConstructionInfo(env);
}

void TrajoptPickAndPlaceConstructor::addLinearMotion(Affine3d start_pose, Affine3d end_pose, int num_steps, int first_time_step)
{
  // linear delta
  Vector3d xyz_delta = (end_pose.translation() - start_pose.translation())/(num_steps - 1);

  Quaterniond approach_rotation(start_pose.linear());
  Matrix3d rotation_diff = (start_pose.linear().inverse() * end_pose.linear());
  AngleAxisd aa_rotation_diff(rotation_diff);
  double angle_delta = aa_rotation_diff.angle()/(num_steps - 1);
  Vector3d delta_axis = aa_rotation_diff.axis();

  // Constraints for linear pick motion
  for (int i = 0; i < num_steps; i++)
  {
    std::shared_ptr<StaticPoseCostInfo> pose_constraint = std::shared_ptr<StaticPoseCostInfo>(new StaticPoseCostInfo);
    pose_constraint->term_type = TT_CNT;
    pose_constraint->link = ee_link_;
    pose_constraint->timestep = i + first_time_step;
    pose_constraint->xyz = start_pose.translation() + xyz_delta * i;

    Quaterniond rotation_delta(cos(0.5*angle_delta*i), delta_axis.x()*sin(0.5*angle_delta*i),
                               delta_axis.y()*sin(0.5*angle_delta*i), delta_axis.z()*sin(0.5*angle_delta*i));
    Quaterniond rotation = rotation_delta * approach_rotation;
    pose_constraint->wxyz = Vector4d(rotation.w(), rotation.x(), rotation.y(), rotation.z());
    pose_constraint->pos_coeffs = Vector3d(10.0, 10.0, 10.0);
    pose_constraint->rot_coeffs = Vector3d(10.0, 10.0, 10.0);
    pose_constraint->name = "pose_" + std::to_string(i + first_time_step);
    pci_.cnt_infos.push_back(pose_constraint);
  }
}

void TrajoptPickAndPlaceConstructor::addInitialJointPosConstraint()
{
  std::shared_ptr<JointConstraintInfo> start_constraint = std::shared_ptr<JointConstraintInfo>(new JointConstraintInfo);
  start_constraint->term_type = TT_CNT;
  start_constraint->timestep = 0;
  start_constraint->name = "start_pos_constraint";
  Eigen::VectorXd start_joint_pos = pci_.env->getCurrentJointValues();
  start_constraint->vals = std::vector<double>(start_joint_pos.data(), start_joint_pos.data() + start_joint_pos.rows());
  pci_.cnt_infos.push_back(start_constraint);
}


void TrajoptPickAndPlaceConstructor::addJointVelCost(double coeff)
{
  std::vector<std::string> joint_names = pci_.kin->getJointNames();
  for (std::size_t i = 0; i < joint_names.size(); i++)
  {
    std::shared_ptr<JointVelTermInfo> jv(new JointVelTermInfo);
    jv->coeffs = std::vector<double>(1, 5.0);
    jv->name = joint_names[i] + "_vel";
    jv->term_type = TT_COST;
    jv->first_step = 0;
    jv->last_step = pci_.basic_info.n_steps - 1;
    jv->joint_name = joint_names[i];
    jv->penalty_type = sco::SQUARED;
    pci_.cost_infos.push_back(jv);
  }
}

void TrajoptPickAndPlaceConstructor::addCollisionCost(double dist_pen, double coeff, int first_step, int last_step)
{
  std::shared_ptr<CollisionCostInfo> collision(new CollisionCostInfo);
  collision->name = "collision";
  collision->term_type = TT_COST;
  collision->continuous = false;
  collision->first_step = first_step;
  collision->last_step = last_step;
  collision->gap = 1;
  collision->info = createSafetyMarginDataVector(last_step - first_step + 1, dist_pen, coeff);

  pci_.cost_infos.push_back(collision);
}
