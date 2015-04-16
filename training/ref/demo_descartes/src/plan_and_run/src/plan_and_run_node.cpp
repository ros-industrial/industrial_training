/*
 * plan_and_run_node.cpp
 *
 *  Created on: Apr 10, 2015
 *      Author: Jorge Nicho
 */

#include <plan_and_run/demo_application.h>

int main(int argc,char** argv)
{
  ros::init(argc,argv,"plan_and_run");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // creating application
  plan_and_run::DemoApplication application;

  // loading parameters
  application.loadParameters();

  // initializing descartes
  application.initDescartes();

  // initializing moveit
  application.initMoveitClient();

  // moving to home position
  application.moveHome();

  // generating trajectory
  plan_and_run::DescartesTrajectory traj;
  application.generateTrajectory(traj);


  // planning robot path
  plan_and_run::DescartesTrajectory output_path;
  application.planPath(traj,output_path);

  // running robot path
  application.runPath(output_path);

  // exiting ros node
  spinner.stop();



  return 0;
}


