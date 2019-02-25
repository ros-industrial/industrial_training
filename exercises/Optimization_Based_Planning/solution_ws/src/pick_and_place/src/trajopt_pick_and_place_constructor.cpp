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
  : manipulator_(manipulator), ee_link_(ee_link), pick_object_(pick_object), tcp_(tcp), env_(env)
{
  kin_ = env->getManipulator(manipulator_);
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

void TrajoptPickAndPlaceConstructor::addSingleWaypoint(trajopt::ProblemConstructionInfo& pci,
                                                       Isometry3d pose,
                                                       int time_step)
{
  Quaterniond rotation(pose.linear());

  std::shared_ptr<CartPoseTermInfo> pose_constraint = std::shared_ptr<CartPoseTermInfo>(new CartPoseTermInfo);
  pose_constraint->term_type = TT_CNT;
  pose_constraint->link = ee_link_;
  pose_constraint->timestep = time_step;
  pose_constraint->xyz = pose.translation();

  pose_constraint->wxyz = Vector4d(rotation.w(), rotation.x(), rotation.y(), rotation.z());
  pose_constraint->pos_coeffs = Vector3d(10.0, 10.0, 10.0);
  pose_constraint->rot_coeffs = Vector3d(10.0, 10.0, 10.0);
  pose_constraint->name = "pose_" + std::to_string(time_step);
  pci.cnt_infos.push_back(pose_constraint);
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
    std::shared_ptr<CartPoseTermInfo> pose_constraint = std::shared_ptr<CartPoseTermInfo>(new CartPoseTermInfo);
    pose_constraint->term_type = TT_CNT;
    pose_constraint->link = ee_link_;
    pose_constraint->timestep = i + first_time_step;
    pose_constraint->xyz = start_pose.translation() + xyz_delta * i;

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
    pose_constraint->wxyz = Vector4d(rotation.w(), rotation.x(), rotation.y(), rotation.z());
    pose_constraint->pos_coeffs = Vector3d(10.0, 10.0, 10.0);
    pose_constraint->rot_coeffs = Vector3d(10.0, 10.0, 10.0);
    pose_constraint->name = "pose_" + std::to_string(i + first_time_step);
    pci.cnt_infos.push_back(pose_constraint);
  }
}

TrajOptProbPtr TrajoptPickAndPlaceConstructor::generatePickProblem(Isometry3d& approach_pose,
                                                                   Isometry3d& final_pose,
                                                                   int steps_per_phase)
{
  //---------------------------------------------------------
  // ---------------- Fill Basic Info -----------------------
  //---------------------------------------------------------
  trajopt::ProblemConstructionInfo pci(env_);
  // Add kinematics
  pci.kin = kin_;

  /* Fill Code: Define the basic info
       . Set the pci number of steps
       . Set the start_fixed to false
       . Set the manipulator name (see class members)
       . Set dt lower limit
       . Set dt upper limit
       . Set use_time to false
  */
  /* ========  ENTER CODE HERE ======== */
  pci.basic_info.n_steps = steps_per_phase * 2;
  pci.basic_info.manip = manipulator_;
  pci.basic_info.dt_lower_lim = 0.2;
  pci.basic_info.dt_upper_lim = .5;
  pci.basic_info.start_fixed = true;
  pci.basic_info.use_time = false;

  //---------------------------------------------------------
  // ---------------- Fill Init Info ------------------------
  //---------------------------------------------------------

  // To use JOINT_INTERPOLATED - a linear interpolation of joint values
  //  pci.init_info.type = InitInfo::JOINT_INTERPOLATED;
  //  pci.init_info.data = numericalIK(final_pose);  // Note, take the last value off if using time (just want jnt
  //  values)

  // To use STATIONARY - all jnts initialized to the starting value
  pci.init_info.type = InitInfo::STATIONARY;
  pci.init_info.dt = 0.5;

  //---------------------------------------------------------
  // ---------------- Fill Term Infos -----------------------
  //---------------------------------------------------------

  // ================= Collision cost =======================
  std::shared_ptr<CollisionTermInfo> collision(new CollisionTermInfo);
  /* Fill Code:
       . Define the cost name
       . Define the term type (This is a cost)
       . Define this cost as continuous
       . Define the first time step
       . Define the last time step
       . Set the cost gap to be 1
  */
  /* ========  ENTER CODE HERE ======== */
  collision->name = "collision";
  collision->term_type = TT_COST;
  collision->continuous = true;
  collision->first_step = 0;
  collision->last_step = pci.basic_info.n_steps - 1;
  collision->gap = 1;
  collision->info = createSafetyMarginDataVector(pci.basic_info.n_steps, 0.025, 40);

  pci.cost_infos.push_back(collision);

  // ================= Velocity cost =======================
  std::shared_ptr<JointVelTermInfo> jv(new JointVelTermInfo);

  // Taken from iiwa documentation (radians/s) and scaled by 0.8
  std::vector<double> vel_lower_lim{ 1.71 * -0.8, 1.71 * -0.8, 1.75 * -0.8, 2.27 * -0.8,
                                     2.44 * -0.8, 3.14 * -0.8, 3.14 * -0.8 };
  std::vector<double> vel_upper_lim{
    1.71 * 0.8, 1.71 * 0.8, 1.75 * 0.8, 2.27 * 0.8, 2.44 * 0.8, 3.14 * 0.8, 3.14 * 0.8
  };

  /* Fill Code:
       . Define the term time (This is a cost)
       . Define the first time step
       . Define the last time step
       . Define vector of target velocities. Length = DOF. Value = 0
       . Define vector of coefficients. Length = DOF. Value = 5
       . Define the term name
  */
  /* ========  ENTER CODE HERE ======== */
  jv->term_type = TT_COST;
  jv->first_step = 0;
  jv->last_step = pci.basic_info.n_steps - 1;
  jv->targets = std::vector<double>(7, 0.0);
  jv->coeffs = std::vector<double>(7, 5.0);
  jv->name = "joint_velocity_cost";

  pci.cost_infos.push_back(jv);

  // ================= Path waypoints =======================
  this->addLinearMotion(pci, approach_pose, final_pose, steps_per_phase, steps_per_phase);

  TrajOptProbPtr result = ConstructProblem(pci);
  return result;
}

TrajOptProbPtr TrajoptPickAndPlaceConstructor::generatePlaceProblem(Isometry3d& retreat_pose,
                                                                    Isometry3d& approach_pose,
                                                                    Isometry3d& final_pose,
                                                                    int steps_per_phase)
{
  //---------------------------------------------------------
  // ---------------- Fill Basic Info -----------------------
  //---------------------------------------------------------
  trajopt::ProblemConstructionInfo pci(env_);
  // Add kinematics
  pci.kin = kin_;

  /* Fill Code: Define the basic info
       . Set the pci number of steps
       . Set the manipulator name (see class members)
       . Set dt lower limit
       . Set dt upper limit
       . Set the start_fixed to false
       . Set use_time to false
  */
  /* ========  ENTER CODE HERE ======== */
  pci.basic_info.n_steps = steps_per_phase * 3;
  pci.basic_info.manip = manipulator_;
  pci.basic_info.dt_lower_lim = 0.005;
  pci.basic_info.dt_upper_lim = .5;
  pci.basic_info.start_fixed = true;
  pci.basic_info.use_time = false;

  //---------------------------------------------------------
  // ---------------- Fill Init Info ------------------------
  //---------------------------------------------------------

  // To use JOINT_INTERPOLATED - a linear interpolation of joint values
  //  pci.init_info.type = InitInfo::JOINT_INTERPOLATED;
  //  pci.init_info.data = numericalIK(final_pose);  // Note, take the last value off if using time (just want jnt
  //  values)

  // To use STATIONARY - all jnts initialized to the starting value
  pci.init_info.type = InitInfo::STATIONARY;
  pci.init_info.dt = 0.5;

  //---------------------------------------------------------
  // ---------------- Fill Term Infos -----------------------
  //---------------------------------------------------------

  // ================= Collision cost =======================
  std::shared_ptr<CollisionTermInfo> collision(new CollisionTermInfo);
  /* Fill Code:
       . Define the cost name
       . Define the term type (This is a cost)
       . Define this cost as continuous
       . Define the first time step
       . Define the last time step
       . Set the cost gap to be 1
  */
  /* ========  ENTER CODE HERE ======== */
  collision->name = "collision";
  collision->term_type = TT_COST;
  collision->continuous = true;
  collision->first_step = 0;
  collision->last_step = pci.basic_info.n_steps - 1;
  collision->gap = 1;
  collision->info = createSafetyMarginDataVector(pci.basic_info.n_steps, 0.025, 40);

  pci.cost_infos.push_back(collision);

  // ================= Velocity cost =======================
  std::shared_ptr<JointVelTermInfo> jv(new JointVelTermInfo);

  // Taken from iiwa documentation (radians/s) and scaled by 0.8
  std::vector<double> vel_lower_lim{ 1.71 * -0.8, 1.71 * -0.8, 1.75 * -0.8, 2.27 * -0.8,
                                     2.44 * -0.8, 3.14 * -0.8, 3.14 * -0.8 };
  std::vector<double> vel_upper_lim{
    1.71 * 0.8, 1.71 * 0.8, 1.75 * 0.8, 2.27 * 0.8, 2.44 * 0.8, 3.14 * 0.8, 3.14 * 0.8
  };

  /* Fill Code:
       . Define the term time (This is a cost)
       . Define the first time step
       . Define the last time step
       . Define vector of target velocities. Length = DOF. Value = 0
       . Define vector of coefficients. Length = DOF. Value = 5
       . Define the term name
  */
  /* ========  ENTER CODE HERE ======== */
  jv->term_type = TT_COST;
  jv->first_step = 0;
  jv->last_step = pci.basic_info.n_steps - 1;
  jv->targets = std::vector<double>(7, 0.0);
  jv->coeffs = std::vector<double>(7, 5.0);
  jv->name = "joint_velocity_cost";

  pci.cost_infos.push_back(jv);

  // Get current pose
  Eigen::Isometry3d start_pose;
  pci.kin->calcFwdKin(start_pose,
                      env_->getState()->transforms.at(kin_->getBaseLinkName()),
                      env_->getCurrentJointValues(),
                      ee_link_,
                      *env_->getState());

  /* Fill Code: Define motion
       . Add linear motion from start_pose to retreat_pose (hint: Use the helpers defined above)
       . Add linear motion from approach_pose to final_pose
  */
  /* ========  ENTER CODE HERE ======== */
  this->addLinearMotion(pci, start_pose, retreat_pose, steps_per_phase, 0);

  this->addLinearMotion(pci, approach_pose, final_pose, steps_per_phase, steps_per_phase * 2);

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
  pci_ik.basic_info.start_fixed = true;
  pci_ik.init_info.type = InitInfo::STATIONARY;
  pci_ik.kin = kin_;
  pci_ik.init_info.data = env_->getCurrentJointValues(pci_ik.kin->getName());

  // Set velocity cost to get "shortest" ik solution
  std::shared_ptr<JointVelTermInfo> jv(new JointVelTermInfo);
  jv->coeffs = std::vector<double>(7, 5.0);
  jv->term_type = TT_COST;
  jv->first_step = 0;
  jv->last_step = pci_ik.basic_info.n_steps - 1;
  jv->name = "joint_velocity_cost";
  pci_ik.cost_infos.push_back(jv);

  // Set static pose constraint
  std::shared_ptr<CartPoseTermInfo> pose_constraint = std::shared_ptr<CartPoseTermInfo>(new CartPoseTermInfo);
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
  sco::BasicTrustRegionSQP opt(problem);
  opt.initialize(DblVec(problem->GetNumDOF() * 2, 0));
  opt.optimize();

  // Get the joint values for only the last point from the output and return them
  Eigen::VectorXd output = util::toVectorXd(opt.x());
  return output.segment(problem->GetNumDOF(), problem->GetNumDOF());
}
