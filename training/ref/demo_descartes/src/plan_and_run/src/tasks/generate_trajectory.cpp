#include <plan_and_run/demo_application.h>

/* GENERATE TRAJECTORY
  Goal:
    - Create a Descartes Trajectory from an array of poses.
    - Create trajectory points that are free to rotate about the tool's z axis
*/

namespace plan_and_run
{

void DemoApplication::generateTrajectory(DescartesTrajectory& traj)
{
  //ROS_ERROR_STREAM("Task '"<<__FUNCTION__ <<"' is incomplete. Exiting"); exit(-1);

  using namespace descartes_core;
  using namespace descartes_trajectory;


  // generating trajectory using a lemniscate curve function.
  EigenSTL::vector_Affine3d poses;
  Eigen::Vector3d center(config_.center[0],config_.center[1],config_.center[2]);
  if(createLemniscateCurve(config_.foci_distance,config_.radius,config_.num_points,
                        config_.num_lemniscates,center,poses))
  {
    ROS_INFO_STREAM("Trajectory with "<<poses.size()<<" points was generated");
  }
  else
  {
    ROS_ERROR_STREAM("Trajectory generation failed");
    exit(-1);
  }

  // publishing trajectory poses for visualization
  publishPosesMarkers(poses);


  // creating descartes trajectory points
  traj.clear();
  traj.reserve(poses.size());
  for(unsigned int i = 0; i < poses.size(); i++)
  {
    const Eigen::Affine3d& pose = poses[i];

    /*  Fill Code:
     * Goal:
     * - Create AxialSymetricPt objects in order to define a trajectory cartesian point with
     *    rotational freedom about the tool's z axis.
     *
     * Hint:
     * - The point can be constructed as follows:
     *
     *    new AxialSymmetricPt(Pose ,Increment, Free Axis)
     *
     * - The Pose can be found in the for loop's "pose" variable.
     * - The Increment can be found in the "ORIENTATION_INCREMENT" global variable.
     * - The Free Axis can be selected from the AxialSymmetricPt::FreeAxis::Z_AXIS enumeration constants.
     *
     */
    //descartes_core::TrajectoryPtPtr pt = descartes_core::TrajectoryPtPtr(/*[ COMPLETE HERE*/);

    descartes_core::TrajectoryPtPtr pt = descartes_core::TrajectoryPtPtr(
        new descartes_trajectory::AxialSymmetricPt(pose,ORIENTATION_INCREMENT,
                                                   descartes_trajectory::AxialSymmetricPt::FreeAxis::Z_AXIS) );

    // saving points into trajectory
    traj.push_back(pt);
  }

  ROS_INFO_STREAM("Task '"<<__FUNCTION__<<"' completed");

}

}


