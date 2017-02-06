/*********************************************************************
 * Copyright (c) 2014, Southwest Research Institute
 * All rights reserved
 *********************************************************************/
#include <pluginlib/class_list_macros.h>
#include <descartes_planner/dense_planner.h>
#include <descartes_planner/sparse_planner.h>

PLUGINLIB_EXPORT_CLASS(descartes_planner::DensePlanner, descartes_core::PathPlannerBase)
PLUGINLIB_EXPORT_CLASS(descartes_planner::SparsePlanner, descartes_core::PathPlannerBase)
