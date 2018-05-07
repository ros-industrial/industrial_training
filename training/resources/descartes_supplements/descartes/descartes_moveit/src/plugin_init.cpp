/*********************************************************************
 * Copyright (c) 2014, Southwest Research Institute
 * All rights reserved
 *********************************************************************/
#include <pluginlib/class_list_macros.h>
#include <descartes_moveit/moveit_state_adapter.h>
#include <descartes_moveit/ikfast_moveit_state_adapter.h>

PLUGINLIB_EXPORT_CLASS(descartes_moveit::MoveitStateAdapter, descartes_core::RobotModel)
PLUGINLIB_EXPORT_CLASS(descartes_moveit::IkFastMoveitStateAdapter, descartes_core::RobotModel)
