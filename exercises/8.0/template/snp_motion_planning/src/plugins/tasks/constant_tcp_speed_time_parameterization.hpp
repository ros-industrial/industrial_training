#pragma once

#include <kdl/chainjnttojacdotsolver.hpp>
#include <kdl/path_line.hpp>
#include <kdl/path_composite.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/velocityprofile_traphalf.hpp>
#include <kdl/utilities/error.h>
#include <tesseract_kinematics/core/joint_group.h>
#include <tesseract_kinematics/kdl/kdl_utils.h>
#include <tesseract_kinematics/core/utils.h>
#include <tesseract_time_parameterization/core/trajectory_container.h>
#include <tesseract_environment/environment.h>

/** @details Adapted from http://docs.ros.org/en/indigo/api/eigen_conversions/html/eigen__kdl_8cpp_source.html#l00090 */
template <typename T>
KDL::Frame toKDL(const T& e)
{
  KDL::Frame k;
  for (unsigned int i = 0; i < 3; ++i)
    k.p[i] = e(i, 3);
  for (unsigned int i = 0; i < 9; ++i)
    k.M.data[i] = e(i / 3, i % 3);

  return k;
}

Eigen::VectorXd fromKDL(const KDL::Twist& k)
{
  Eigen::VectorXd out(6);
  for (int i = 0; i < 6; ++i)
    out[i] = k[i];
  return out;
}

namespace snp_motion_planning
{
class ConstantTCPSpeedTimeParameterization
{
public:
  using Ptr = std::shared_ptr<ConstantTCPSpeedTimeParameterization>;
  using ConstPtr = std::shared_ptr<const ConstantTCPSpeedTimeParameterization>;

  ConstantTCPSpeedTimeParameterization(tesseract_environment::Environment::ConstPtr env, const std::string& group,
                                       const std::string& tcp, const double max_translational_vel,
                                       const double max_rotational_vel, const double max_translational_acc,
                                       const double max_rotational_acc)
    : motion_group(env->getJointGroup(group))
    , tcp(tcp)
    , max_translational_vel(max_translational_vel)
    , max_translational_acc(max_translational_acc)
    , eq_radius_(std::max((max_translational_vel / max_rotational_vel), (max_translational_acc / max_rotational_acc)))
  {
    // Construct the KDL chain
    tesseract_kinematics::KDLChainData data;
    if (!tesseract_kinematics::parseSceneGraph(data, *env->getSceneGraph(), motion_group->getBaseLinkName(), tcp))
      throw std::runtime_error("Failed to construct KDL chain");

    kdl_chain_ = data.robot_chain;
  }

  bool compute(tesseract_planning::TrajectoryContainer& trajectory, double max_velocity_scaling_factor = 1.0,
               double max_acceleration_scaling_factor = 1.0) const
  {
    try
    {
      auto path = new KDL::Path_Composite();

      // Create a container for the times of each waypoint in the newly parameterized trajectory
      std::vector<double> times;
      times.reserve(trajectory.size());
      times.push_back(0.0);

      const double max_vel = max_velocity_scaling_factor * max_translational_vel;
      const double max_acc = max_acceleration_scaling_factor * max_translational_acc;
      KDL::VelocityProfile_TrapHalf v_trap_half(max_vel, max_acc, true);
      auto v_trap = new KDL::VelocityProfile_Trap(max_vel, max_acc);

      for (Eigen::Index i = 1; i < trajectory.size(); ++i)
      {
        const Eigen::VectorXd& start_joints = trajectory.getPosition(i - 1);
        const Eigen::VectorXd& end_joints = trajectory.getPosition(i);

        // Perform FK to get Cartesian poses
        const KDL::Frame start = toKDL(motion_group->calcFwdKin(start_joints).at(tcp));
        const KDL::Frame end = toKDL(motion_group->calcFwdKin(end_joints).at(tcp));

        // Convert to KDL::Path
        KDL::RotationalInterpolation* rot_interp = new KDL::RotationalInterpolation_SingleAxis();
        KDL::Path* segment = new KDL::Path_Line(start, end, rot_interp, eq_radius_);

        // Add the segment to the full path
        path->Add(segment);

        double path_length = path->PathLength() > std::numeric_limits<double>::epsilon() ?
                                 path->PathLength() :
                                 std::numeric_limits<double>::epsilon();

        /* KDL does not provide the ability to determine the time at which a particular waypoint occurs in velocity
         * profile parameterized trajectory. Therefore, we need to estimate the time to each waypoint. We can do this by
         * applying a half-trapezoid (front) velocity profile to the path constructed so far and calculating the
         * duration.
         */
        v_trap_half.SetProfile(0.0, path_length);
        times.push_back(v_trap_half.Duration());
      }

      // Apply a double-ended trapezoidal velocity profile to the full path
      double path_length = path->PathLength();
      v_trap->SetProfile(0.0, path_length);
      const double duration = v_trap->Duration();

      // Add the last time with the duration from a double ended trapezoidal velocity profile
      times.back() = duration;

      /* In some cases, the deceleration may not occur completely in the last path segment, so estimating the time to a
       * waypoint using the forward half-trapezoidal velocity profile can be incorrect. Instead we need to work
       * backwards from the end of the trajectory to each waypoint until we reach the halfway point or until we reach a
       * waypoint that is at full speed
       */
      {
        // Construct a reverse path
        KDL::Path_Composite reverse_path;
        int n_segments = path->GetNrOfSegments();
        int middle_segment = n_segments / 2;

        // Iterate in reverse through the path until we hit a point whose velocity is
        for (int i = path->GetNrOfSegments(); i-- > middle_segment;)
        {
          reverse_path.Add(path->GetSegment(i), false);

          double reverse_path_length = reverse_path.PathLength() > std::numeric_limits<double>::epsilon() ?
                                           reverse_path.PathLength() :
                                           std::numeric_limits<double>::epsilon();
          v_trap_half.SetProfile(0.0, reverse_path_length);
          double reverse_path_duration = v_trap_half.Duration();
          double vel = v_trap_half.Vel(reverse_path_duration);

          if (std::abs(vel - max_vel) > std::numeric_limits<double>::epsilon())
          {
            times[i] = duration - reverse_path_duration;
          }
          else
          {
            break;
          }
        }
      }

      // Update the trajectory
      for (Eigen::Index i = 0; i < trajectory.size(); ++i)
      {
        const Eigen::VectorXd& joints = trajectory.getPosition(i);
        double t = times[i];

        // Compute the joint velocity and acceleration
        KDL::Trajectory_Segment traj(path, v_trap, false);
        KDL::Twist vel = traj.Vel(t);
        KDL::Twist acc = traj.Acc(t);
        const Eigen::VectorXd joint_vel = computeJointVelocity(vel, joints);
        const Eigen::VectorXd joint_acc = computeJointAcceleration(acc, joints, joint_vel);

        // Update the trajectory container
        trajectory.setData(i, joint_vel, joint_acc, t);
      }
    }
    catch (KDL::Error& e)
    {
      std::stringstream ss;
      ss << "KDL Error #" << e.GetType() << ": " << e.Description();
      CONSOLE_BRIDGE_logError(ss.str().c_str());
      return false;
    }
    catch (const std::exception& ex)
    {
      CONSOLE_BRIDGE_logError(ex.what());
      return false;
    }

    return true;
  }

private:
  Eigen::VectorXd computeJointVelocity(const KDL::Twist& x_dot, const Eigen::VectorXd& q) const
  {
    Eigen::MatrixXd jac = motion_group->calcJacobian(q, tcp);

    Eigen::Index dof = q.size();
    Eigen::VectorXd q_dot(dof);
    if (dof == 6)
    {
      q_dot = jac.colPivHouseholderQr().solve(fromKDL(x_dot));
    }
    else
    {
      if (!tesseract_kinematics::solvePInv(jac, fromKDL(x_dot), q_dot))
        throw std::runtime_error("Failed to solve pseudo-inverse for joint velocity calculation");
    }

    return q_dot;
  }

  Eigen::VectorXd computeJointAcceleration(const KDL::Twist& x_dot_dot, const Eigen::VectorXd& q,
                                           const Eigen::VectorXd& q_dot) const
  {
    // Create a Jacobian derivative solver
    KDL::ChainJntToJacDotSolver solver(kdl_chain_);

    // Compute the derivative of the Jacobian
    KDL::JntArrayVel jnt_array;
    jnt_array.q.data = q;
    jnt_array.qdot.data = q_dot;
    KDL::Twist jac_dot_q_dot;
    {
      int error = solver.JntToJacDot(jnt_array, jac_dot_q_dot);
      if (error != 0)
        throw std::runtime_error(solver.strError(error));
    }

    // Compute the jacbian
    Eigen::MatrixXd jac = motion_group->calcJacobian(q, tcp);

    // Compute the joint accelerations
    // d/dt(x_dot) = d/dt(J * q_dot)
    // x_dot_dot = J_dot * q_dot + J * q_dot_dot
    // qdotdot = J^(-1) * (x_dot_dot - J_dot_q_dot)
    Eigen::VectorXd twist = fromKDL(x_dot_dot - jac_dot_q_dot);

    Eigen::Index dof = q.size();
    Eigen::VectorXd q_dot_dot(dof);
    if (dof == 6)
    {
      q_dot_dot = jac.colPivHouseholderQr().solve(twist);
    }
    else
    {
      if (!tesseract_kinematics::solvePInv(jac, twist, q_dot_dot))
        throw std::runtime_error("Failed to solve pseudo-inverse for acceleration calculation");
    }

    return q_dot_dot;
  }

  tesseract_kinematics::JointGroup::UPtr motion_group;
  KDL::Chain kdl_chain_;
  const std::string tcp;
  const double max_translational_vel;
  const double max_translational_acc;
  const double eq_radius_;
};

}  // namespace snp_motion_planning
