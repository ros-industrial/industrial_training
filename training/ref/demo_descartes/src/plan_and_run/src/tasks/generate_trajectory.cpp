#include <plan_and_run/demo_application.h>

namespace plan_and_run
{

void DemoApplication::generateTrajectory(DescartesTrajectory& traj)
{

  using namespace descartes_core;
  using namespace descartes_trajectory;

  // generating lemniscate curve
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

    // creating cartesian point with rotational freedom about the tool's approach axis
    descartes_core::TrajectoryPtPtr pt = descartes_core::TrajectoryPtPtr(
        new descartes_trajectory::AxialSymmetricPt(pose,ORIENTATION_INCREMENT,
                                                   descartes_trajectory::AxialSymmetricPt::FreeAxis::Z_AXIS) );

    traj.push_back(pt);
  }

}

}


