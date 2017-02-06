/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2014, Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/*
 * planning_graph.cpp
 *
 *  Created on: Jun 5, 2014
 *      Author: Dan Solomon
 */

#include "descartes_planner/planning_graph.h"

#include <stdio.h>
#include <iomanip>
#include <iostream>
#include <utility>
#include <algorithm>
#include <fstream>

#include <ros/console.h>

#include <boost/graph/dijkstra_shortest_paths.hpp>

using namespace descartes_core;
using namespace descartes_trajectory;
namespace descartes_planner
{

const double MAX_JOINT_DIFF = M_PI / 2;
const double MAX_EXCEEDED_PENALTY = 10000.0f;

PlanningGraph::PlanningGraph(RobotModelConstPtr model)
  : robot_model_(std::move(model))
  , cartesian_point_link_(NULL)
{}

PlanningGraph::~PlanningGraph()
{
  delete cartesian_point_link_;
}

CartesianMap PlanningGraph::getCartesianMap()
{
  TrajectoryPt::ID cart_id = descartes_core::TrajectoryID::make_nil();
  for(std::map<TrajectoryPt::ID, CartesianPointInformation>::iterator c_iter = cartesian_point_link_->begin();
      c_iter != cartesian_point_link_->end(); c_iter++)
  {
    ROS_DEBUG_STREAM("Checking for last TrajectoryID: " << c_iter->first);
    if(c_iter->second.links_.id_next.is_nil())
    {
      cart_id = c_iter->first;
      break;
    }
  }

  CartesianMap to_return = CartesianMap();
  bool done = false;
  while(!done)
  {
    to_return[cart_id] = (*cartesian_point_link_)[cart_id];
    cart_id = (*cartesian_point_link_)[cart_id].links_.id_previous;
    done = (cart_id.is_nil());
    ROS_DEBUG_STREAM("Next CID: " << cart_id);
  }

  return to_return;
}

descartes_core::RobotModelConstPtr PlanningGraph::getRobotModel()
{
  return robot_model_;
}

bool PlanningGraph::insertGraph(const std::vector<TrajectoryPtPtr> *points)
{
  // validate input
  if (!points)
  {
    // one or both are null
    ROS_ERROR_STREAM("points == null. Cannot initialize graph with null list.");
    return false;
  }
  if (points->size() == 0)
  {
    // one or both have 0 elements
    ROS_ERROR_STREAM("points.size == 0. Cannot initialize graph with 0 elements.");
    return false;
  }

  delete cartesian_point_link_; // remove the previous map before we start on a new one
  cartesian_point_link_ = new std::map<TrajectoryPt::ID, CartesianPointInformation>();

  // DEBUG
  //printMaps();
  TrajectoryPt::ID previous_id = descartes_core::TrajectoryID::make_nil();

  // input is valid, copy to local maps that will be maintained by the planning graph
  for (std::vector<TrajectoryPtPtr>::const_iterator point_iter = points->begin();
      point_iter != points->end(); point_iter++)
  {
    (*cartesian_point_link_)[point_iter->get()->getID()].source_trajectory_ = (*point_iter);
    CartesianPointRelationship point_link = CartesianPointRelationship();
    point_link.id = point_iter->get()->getID();
    point_link.id_next = descartes_core::TrajectoryID::make_nil(); // default to nil UUID
    point_link.id_previous = descartes_core::TrajectoryID::make_nil(); // default to nil UUID

    // if the previous_id exists, set it's next_id to the new id
    if (cartesian_point_link_->find(previous_id) != cartesian_point_link_->end())
    {
      (*cartesian_point_link_)[previous_id].links_.id_next = point_link.id;
      
      ROS_DEBUG_STREAM("PreviousID[" << previous_id << "].links_.id_next = " << point_link.id);
    }
    else
    {
      ROS_INFO_STREAM("PreviousID: " << previous_id << " was not found");
    }

    // set the new current point link
    point_link.id_previous = previous_id;

    // the new one becomes the previous_id
    previous_id = point_link.id;

    // save the point_link structure to the map
    (*cartesian_point_link_)[point_link.id].links_ = point_link;
  }

  // after populating maps above (presumably from cartesian trajectory points), calculate (or query) for all joint trajectories
  if (!calculateJointSolutions())
  {
    // failed to get joint trajectories
    ROS_ERROR_STREAM("unable to calculate joint trajectories for input points");
    return false;
  }

  if (!populateGraphVertices())
  {
    ROS_ERROR_STREAM("unable to populate graph from input points");
    return false;
  }

  // after obtaining joint trajectories, calculate each edge weight between adjacent joint trajectories
  // edge list can be local here, it needs to be passed to populate the graph but not maintained afterwards
  std::list<JointEdge> edges;
  if (!calculateAllEdgeWeights(edges))
  {
    // failed to get edge weights
    ROS_ERROR_STREAM("unable to calculate edge weight of joint transitions for joint trajectories");
    return false;
  }

  // from list of joint trajectories (vertices) and edges (edges), construct the actual graph
  if (!populateGraphEdges(edges))
  {
    // failed to create graph
    ROS_ERROR_STREAM("unable to populate graph from calculated edges");
    return false;
  }

  return true;
}

bool PlanningGraph::addTrajectory(TrajectoryPtPtr point, TrajectoryPt::ID previous_id, TrajectoryPt::ID next_id)
{
  if (previous_id.is_nil() && next_id.is_nil())
  {
    // unable to add a point with no forward or backward connections
    ROS_ERROR_STREAM("unable to add a point that is not connected to one or more existing points");
    return false;
  }

  if (!previous_id.is_nil() && cartesian_point_link_->find(previous_id) == cartesian_point_link_->end())
  {
    // unable to find referenced previous_id
    ROS_ERROR_STREAM("unable to find previous point");
    return false;
  }
  if (!next_id.is_nil() && cartesian_point_link_->find(next_id) == cartesian_point_link_->end())
  {
    // unable to find referenced next_id
    ROS_ERROR_STREAM("unable to find next point");
  }

  CartesianPointRelationship point_link;
  point_link.id = point->getID();
  point_link.id_next = next_id;
  point_link.id_previous = previous_id;

  // save the new point_link structure to the map
  (*cartesian_point_link_)[point_link.id].links_ = point_link;

  // if not adding at the beginning, update the previous to point to the new point
  //if (cartesian_point_link_->find(previous_id) != cartesian_point_link_->end())
  if(!previous_id.is_nil())
  {
    (*cartesian_point_link_)[previous_id].links_.id_next = point_link.id;
  }
  // if not updating the end, update the next to point to the new point
  //if (cartesian_point_link_->find(next_id) != cartesian_point_link_->end())
  if(!next_id.is_nil())
  {
    (*cartesian_point_link_)[next_id].links_.id_previous = point_link.id;
  }

  ROS_DEBUG_STREAM("New ID: " << point_link.id);
  ROS_DEBUG_STREAM("New Next ID: " << point_link.id_next);
  ROS_DEBUG_STREAM("New Previous ID: " << point_link.id_previous);

  std::map<TrajectoryPt::ID, JointGraph::vertex_descriptor> joint_vertex_map;
  int num_joints = recalculateJointSolutionsVertexMap(joint_vertex_map);

  if(!previous_id.is_nil() && !next_id.is_nil())
  {
    // remove graph edges from each joint at [id_previous] to joint at [id_next]
    std::list<TrajectoryPt::ID> start_joint_ids = (*cartesian_point_link_)[previous_id].joints_;
    std::list<TrajectoryPt::ID> end_joint_ids = (*cartesian_point_link_)[next_id].joints_;
    for (std::list<TrajectoryPt::ID>::iterator start_joint_iter = start_joint_ids.begin();
        start_joint_iter != start_joint_ids.end(); start_joint_iter++)
    {
      for (std::list<TrajectoryPt::ID>::iterator end_joint_iter = end_joint_ids.begin();
          end_joint_iter != end_joint_ids.end(); end_joint_iter++)
      {
        ROS_DEBUG_STREAM("Removing edge: " << *start_joint_iter << " -> " << *end_joint_iter);
        boost::remove_edge(joint_vertex_map[*start_joint_iter], joint_vertex_map[*end_joint_iter], dg_);
      }
    }
  }

  // get joint poses from new trajectory point (unique id)
  std::vector<std::vector<double> > joint_poses;
  point.get()->getJointPoses(*robot_model_, joint_poses);

  if (joint_poses.size() == 0)
  {
    ROS_WARN_STREAM("no joint solution for this point... potential discontinuity in the graph");
  }
  else
  {
    for (std::vector<std::vector<double> >::iterator joint_pose_iter = joint_poses.begin();
        joint_pose_iter != joint_poses.end(); joint_pose_iter++)
    {
      //get UUID from JointTrajPt (convert from std::vector<double>)
      JointTrajectoryPt new_pt (*joint_pose_iter);
      (*cartesian_point_link_)[point->getID()].joints_.push_back(new_pt.getID());

      // insert new vertices into graph
      JointGraph::vertex_descriptor v = boost::add_vertex(dg_);
      dg_[v].id = new_pt.getID();

      joint_solutions_map_[new_pt.getID()] = new_pt;
    }
  }

  // save the list of joint solutions
  std::list<TrajectoryPt::ID> traj_solutions = (*cartesian_point_link_)[point->getID()].joints_;
  // save the actual trajectory point into the map
  (*cartesian_point_link_)[point->getID()].source_trajectory_ = point;

  ROS_INFO_STREAM("SAVE CART POINT ID[" << point->getID() << "]: "
          << (*cartesian_point_link_)[point->getID()].source_trajectory_.get()->getID());

  std::list<JointEdge> edges;
  // recalculate edges(previous -> this; this -> next)

  if(!previous_id.is_nil())
  {
    std::list<TrajectoryPt::ID> previous_joint_ids = (*cartesian_point_link_)[previous_id].joints_;
    calculateEdgeWeights(previous_joint_ids, traj_solutions, edges);
  }

  if(!next_id.is_nil())
  {
    std::list<TrajectoryPt::ID> next_joint_ids = (*cartesian_point_link_)[next_id].joints_;
    calculateEdgeWeights(traj_solutions, next_joint_ids, edges);
  }

  // insert new edges
  populateGraphEdges(edges);

  // DEBUG LOGS
//  printGraph();
//  printMaps();

  //CartesianMap test_map = getCartesianMap();

  // simple test for now to see the new point is in the list
  return (cartesian_point_link_->find(point->getID()) != cartesian_point_link_->end());
}

bool PlanningGraph::modifyTrajectory(TrajectoryPtPtr point)
{
  TrajectoryPt::ID modify_id = point.get()->getID();
  ROS_INFO_STREAM("Attempting to modify point:: " << modify_id);

//  printMaps();

  if (modify_id.is_nil())
  {
    // unable to modify a point with nil ID
    ROS_ERROR_STREAM("unable to modify a point with nil ID");
    return false;
  }

  if (cartesian_point_link_->find(modify_id) == cartesian_point_link_->end())
  {
    // unable to find cartesian point with ID:
    ROS_ERROR_STREAM("unable to find cartesian point link with ID: " << modify_id);
    return false;
  }
  else
  {
    ROS_INFO_STREAM("Found ID: " << modify_id << " to modify.");
  }

  std::map<TrajectoryPt::ID, JointGraph::vertex_descriptor> joint_vertex_map;
  int num_joints = recalculateJointSolutionsVertexMap(joint_vertex_map);

  // identify joint points at this cartesian point
  std::list<TrajectoryPt::ID> start_joint_ids = (*cartesian_point_link_)[modify_id].joints_;

  ROS_INFO_STREAM("start_joint_ids.size = " << start_joint_ids.size());

  std::vector<JointGraph::edge_descriptor> to_remove_edges;
  std::vector<JointGraph::vertex_descriptor> to_remove_vertices;
  // remove edges
  for (std::list<TrajectoryPt::ID>::iterator start_joint_iter = start_joint_ids.begin();
      start_joint_iter != start_joint_ids.end(); start_joint_iter++)
  {
    // get the graph vertex descriptor
    JointGraph::vertex_descriptor jv = joint_vertex_map[*start_joint_iter];

    // NOTE: cannot print jv as int when using listS
    ROS_DEBUG_STREAM("jv: " << jv);

    // TODO: make this a standalone function to take a jv and return a list of edges to remove
    // remove out edges
    std::pair<OutEdgeIterator, OutEdgeIterator> out_ei = out_edges(jv, dg_);

    for (OutEdgeIterator out_edge = out_ei.first; out_edge != out_ei.second; ++out_edge)
    {
      JointGraph::edge_descriptor e = *out_edge;
      ROS_DEBUG_STREAM("REMOVE OUTEDGE: " << dg_[e].joint_start << " -> " << dg_[e].joint_end);
      to_remove_edges.push_back(e);
    }

    // remove in edges
    std::pair<InEdgeIterator, InEdgeIterator> in_ei = in_edges(jv, dg_);
    for (InEdgeIterator in_edge = in_ei.first; in_edge != in_ei.second; ++in_edge)
    {
      JointGraph::edge_descriptor e = *in_edge;
      ROS_DEBUG_STREAM("REMOVE INEDGE: " << dg_[e].joint_start << " -> " << dg_[e].joint_end);
      to_remove_edges.push_back(e);
    }

    to_remove_vertices.push_back(jv);
    joint_solutions_map_.erase(*start_joint_iter);
//    printMaps();
  }
  for(std::vector<JointGraph::edge_descriptor>::iterator e_iter = to_remove_edges.begin(); e_iter != to_remove_edges.end(); e_iter++)
  {
    ROS_DEBUG_STREAM("REMOVE EDGE: " << dg_[*e_iter].joint_start << " -> " << dg_[*e_iter].joint_end);
    boost:remove_edge(*e_iter, dg_);
    //printGraph();
  }
  std::sort(to_remove_vertices.begin(), to_remove_vertices.end());
  std::reverse(to_remove_vertices.begin(), to_remove_vertices.end());
  for(std::vector<JointGraph::vertex_descriptor>::iterator v_iter = to_remove_vertices.begin(); v_iter != to_remove_vertices.end(); v_iter++)
  {
    // remove the graph vertex and joint point
    // NOTE: cannot print jv as int when using listS
    ROS_DEBUG_STREAM("REMOVE VERTEX: " << *v_iter);
    boost::remove_vertex(*v_iter, dg_);
//    printGraph();
  }
  (*cartesian_point_link_)[modify_id].joints_.clear();

  // get new joint points for this cartesian
  std::vector<std::vector<double> > joint_poses;
  point.get()->getJointPoses(*robot_model_, joint_poses);

  if (joint_poses.size() == 0)
  {
    ROS_WARN_STREAM("no joint solution for this point... potential discontinuity in the graph");
  }
  else
  {
    for (std::vector<std::vector<double> >::iterator joint_pose_iter = joint_poses.begin();
        joint_pose_iter != joint_poses.end(); joint_pose_iter++)
    {
      //get UUID from JointTrajPt (convert from std::vector<double>)
      JointTrajectoryPt new_pt (*joint_pose_iter);
      (*cartesian_point_link_)[modify_id].joints_.push_back(new_pt.getID());

      // insert new vertices into graph
      JointGraph::vertex_descriptor v = boost::add_vertex(dg_);
      dg_[v].id = new_pt.getID();

      joint_solutions_map_[new_pt.getID()] = new_pt;
      ROS_INFO_STREAM("Added New Joint: " << v);
    }
  }

  std::list<TrajectoryPt::ID> traj_solutions = (*cartesian_point_link_)[modify_id].joints_;
  (*cartesian_point_link_)[modify_id].source_trajectory_ = point;
  // don't need to modify links

  // recalculate edges(previous -> this; this -> next)
  TrajectoryPt::ID previous_cart_id = (*cartesian_point_link_)[modify_id].links_.id_previous; // should be equal to cart_link_iter->second.links_.id
  TrajectoryPt::ID next_cart_id = (*cartesian_point_link_)[modify_id].links_.id_next;

  std::list<JointEdge> edges;
  // recalculate edges(previous -> this; this -> next)

  if(!previous_cart_id.is_nil())
  {
    std::list<TrajectoryPt::ID> previous_joint_ids = (*cartesian_point_link_)[previous_cart_id].joints_;
    ROS_DEBUG_STREAM("Calculating previous -> new weights: " << previous_joint_ids.size() << " -> " << traj_solutions.size());
    calculateEdgeWeights(previous_joint_ids, traj_solutions, edges);
  }

  if(!next_cart_id.is_nil())
  {
    std::list<TrajectoryPt::ID> next_joint_ids = (*cartesian_point_link_)[next_cart_id].joints_;
    ROS_DEBUG_STREAM("Calculating new -> next weights: " << traj_solutions.size() << " -> " << next_joint_ids.size());
    calculateEdgeWeights(traj_solutions, next_joint_ids, edges);
  }

  ROS_INFO_STREAM("NEW EDGES: " << edges.size());
  // insert new edges
  return populateGraphEdges(edges);

  // DEBUG LOGS
//  printGraph();
//  printMaps();
}

bool PlanningGraph::removeTrajectory(TrajectoryPtPtr point)
{
  TrajectoryPt::ID delete_id = point.get()->getID();
  ROS_INFO_STREAM("Attempting to delete ID: " << delete_id);

  if (delete_id.is_nil())
  {
    // unable to modify a point with nil ID
    ROS_ERROR_STREAM("unable to delete a point with nil ID");
    return false;
  }
  if (cartesian_point_link_->find(delete_id) == cartesian_point_link_->end())
  {
    // unable to find cartesian point with ID:
    ROS_ERROR_STREAM("unable to find cartesian point link with ID: " << delete_id);
    return false;
  }
  // identify joint points at this cartesian point
  std::list<TrajectoryPt::ID> start_joint_ids = (*cartesian_point_link_)[delete_id].joints_;

  ROS_INFO_STREAM("Attempting to delete edges from " << start_joint_ids.size() << " vertices");

  std::map<TrajectoryPt::ID, JointGraph::vertex_descriptor> joint_vertex_map;
  int num_joints = recalculateJointSolutionsVertexMap(joint_vertex_map);

  std::vector<JointGraph::edge_descriptor> to_remove_edges;
  std::vector<JointGraph::vertex_descriptor> to_remove_vertices;
  // remove edges
  for (std::list<TrajectoryPt::ID>::iterator start_joint_iter = start_joint_ids.begin();
      start_joint_iter != start_joint_ids.end(); start_joint_iter++)
  {
    // get the graph vertex descriptor
    JointGraph::vertex_descriptor jv = joint_vertex_map[*start_joint_iter];

    // remove out edges
    std::pair<OutEdgeIterator, OutEdgeIterator> out_ei = out_edges(jv, dg_);

    for (OutEdgeIterator out_edge = out_ei.first; out_edge != out_ei.second; ++out_edge)
    {
      JointGraph::edge_descriptor e = *out_edge;
      ROS_DEBUG_STREAM("REMOVE OUTEDGE: " << dg_[e].joint_start << " -> " << dg_[e].joint_end);
      to_remove_edges.push_back(e);
    }

    // remove in edges
    std::pair<InEdgeIterator, InEdgeIterator> in_ei = in_edges(jv, dg_);
    for (InEdgeIterator in_edge = in_ei.first; in_edge != in_ei.second; ++in_edge)
    {
      JointGraph::edge_descriptor e = *in_edge;
      ROS_DEBUG_STREAM("REMOVE INEDGE: " << dg_[e].joint_start << " -> " << dg_[e].joint_end);
      to_remove_edges.push_back(e);
    }
    to_remove_vertices.push_back(jv);
    // remove the graph vertex and joint point

    joint_solutions_map_.erase(*start_joint_iter);
  }

  for(std::vector<JointGraph::edge_descriptor>::iterator e_iter = to_remove_edges.begin(); e_iter != to_remove_edges.end(); e_iter++)
  {
    boost:remove_edge(*e_iter, dg_);
  }

  std::sort(to_remove_vertices.begin(), to_remove_vertices.end());
  std::reverse(to_remove_vertices.begin(), to_remove_vertices.end());
  for(std::vector<JointGraph::vertex_descriptor>::iterator v_iter = to_remove_vertices.begin(); v_iter != to_remove_vertices.end(); v_iter++)
  {
    // remove the graph vertex and joint point
    // NOTE: cannot print jv as int when using listS
    ROS_INFO_STREAM("REMOVE VERTEX: " << *v_iter);
    boost::remove_vertex(*v_iter, dg_);
//    printGraph();
  }
  (*cartesian_point_link_)[delete_id].joints_.clear();

  // get previous_id and next_id from cartesian list
  CartesianPointRelationship links = (*cartesian_point_link_)[delete_id].links_;
  TrajectoryPt::ID previous_id = links.id_previous;
  TrajectoryPt::ID next_id = links.id_next;

  if(!previous_id.is_nil())
  {
    // change previous_cartesian link to next_id
    (*cartesian_point_link_)[previous_id].links_.id_next = next_id;
  }
  if(!next_id.is_nil())
  {
    // change next_cartesian link to previous_id
    (*cartesian_point_link_)[next_id].links_.id_previous = previous_id;
  }

  std::list<JointEdge> edges;
  // recalculate edges(previous -> this; this -> next)

  if(!previous_id.is_nil() && !next_id.is_nil())
  {
    std::list<TrajectoryPt::ID> previous_joint_ids = (*cartesian_point_link_)[previous_id].joints_;
    std::list<TrajectoryPt::ID> next_joint_ids = (*cartesian_point_link_)[next_id].joints_;
    calculateEdgeWeights(previous_joint_ids, next_joint_ids, edges);
  }

  // insert new edges
  populateGraphEdges(edges);

  // DEBUG LOGS
//  printGraph();
//  printMaps();

  return true;
}

bool PlanningGraph::findStartVertices(std::list<JointGraph::vertex_descriptor> &start_points)
{
  // Create local id->vertex map  
  std::map<TrajectoryPt::ID, JointGraph::vertex_descriptor> joint_vertex_map;
  int num_joints = recalculateJointSolutionsVertexMap(joint_vertex_map);

  // Find the TrajectoryPt ID of the first point specified by the user
  TrajectoryPt::ID cart_id = descartes_core::TrajectoryID::make_nil();

  for(std::map<TrajectoryPt::ID, CartesianPointInformation>::iterator c_iter = cartesian_point_link_->begin();
      c_iter != cartesian_point_link_->end(); c_iter++)
  {
    if(c_iter->second.links_.id_previous.is_nil())
    {
      cart_id = c_iter->first;
      break;
    }
  }

  // Test for error case
  if (cart_id.is_nil())
  {
    ROS_ERROR("Could not locate TrajectoryPt with nil previous point. Graph may be cyclic.");
    return false;
  }

  // Iterate through the JointSolutions for the last point
  const std::list<descartes_core::TrajectoryPt::ID>& ids = cartesian_point_link_->at(cart_id).joints_;

  for (const auto& id : ids)
  {
    JointGraph::vertex_descriptor v = joint_vertex_map.at(id); 
    start_points.push_back(v);
  }

  return !start_points.empty();
}

bool PlanningGraph::findEndVertices(std::list<JointGraph::vertex_descriptor> &end_points)
{
  // Create local id->vertex map  
  std::map<TrajectoryPt::ID, JointGraph::vertex_descriptor> joint_vertex_map;
  int num_joints = recalculateJointSolutionsVertexMap(joint_vertex_map);

  // Find the TrajectoryPt ID of the last point specified by the user
  TrajectoryPt::ID cart_id = descartes_core::TrajectoryID::make_nil();

  for(std::map<TrajectoryPt::ID, CartesianPointInformation>::iterator c_iter = cartesian_point_link_->begin();
      c_iter != cartesian_point_link_->end(); c_iter++)
  {
    if(c_iter->second.links_.id_next.is_nil())
    {
      cart_id = c_iter->first;
      break;
    }
  }

  // Test for error case
  if (cart_id.is_nil())
  {
    ROS_ERROR("Could not locate TrajectoryPt with nil next point. Graph may be cyclic.");
    return false;
  }

  // Iterate through the JointSolutions for the last point
  const std::list<descartes_core::TrajectoryPt::ID>& ids = cartesian_point_link_->at(cart_id).joints_;

  for (const auto& id : ids)
  {
    JointGraph::vertex_descriptor v = joint_vertex_map.at(id); 
    end_points.push_back(v);
  }

  return !end_points.empty();
}


bool PlanningGraph::getShortestPath(double &cost, std::list<JointTrajectoryPt> &path)
{
  std::list<JointGraph::vertex_descriptor> start_points;
  findStartVertices(start_points);
  std::list<JointGraph::vertex_descriptor> end_points;
  findEndVertices(end_points);

  size_t num_vert = boost::num_vertices(dg_);

  std::vector<TrajectoryPt::ID> vertex_index_map(num_vert);
  std::pair<VertexIterator, VertexIterator> vi = boost::vertices(dg_);
  int i = 0;
  for (VertexIterator vert_iter = vi.first; vert_iter != vi.second; ++vert_iter)
  {
    JointGraph::vertex_descriptor jv = *vert_iter;
    vertex_index_map[i++] = dg_[jv].id;
  }

  cost = std::numeric_limits<double>::max();

  for (std::list<JointGraph::vertex_descriptor>::iterator start = start_points.begin(); start != start_points.end(); start++)
  {
    // initialize vectors to be used by dijkstra
    std::vector<JointGraph::vertex_descriptor> predecessors(num_vert);
    std::vector<double> weights(num_vert, std::numeric_limits<double>::max());

    dijkstra_shortest_paths(
        dg_,
        *start,
        weight_map(get(&JointEdge::transition_cost, dg_)).distance_map(
          boost::make_iterator_property_map(weights.begin(), get(boost::vertex_index, dg_))).predecessor_map(
          &predecessors[0]));

    for (std::list<JointGraph::vertex_descriptor>::iterator end = end_points.begin(); end != end_points.end(); end++)
    {
      //ROS_INFO("Checking path: S[%d] -> E[%d]", *start, *end);
      // actual weight(cost) from start_id to end_id
      double weight = weights[*end];

      // if the weight of this path of less than a previous one, replace the return path
      if (weight < cost)
      {
        std::stringstream ss;
        ss << "New Shortest Path: ";

        cost = weight;
        path.clear();
        // Add the destination point.
        int current = *end;
        path.push_front(joint_solutions_map_[vertex_index_map[current]]);

        ss << current;
        // Starting from the destination point step through the predecessor map
        // until the source point is reached.
        while (current != *start)
        {
          current = predecessors[current];
          ss << " -> " << current;
          path.push_front(joint_solutions_map_[vertex_index_map[current]]);
        }
        ROS_INFO("With %lu - new best score: %f", path.size(), weight);
      }
    }
  }

  if (cost < std::numeric_limits<double>::max())
  {
    return true;
  }
  else
  {
    ROS_ERROR_STREAM("unable to find a valid path");
    return false;
  }
}

void PlanningGraph::printMaps()
{
  ROS_DEBUG_STREAM("Number of points: " << cartesian_point_link_->size());

  for(std::map<TrajectoryPt::ID, CartesianPointInformation>::iterator c_iter = cartesian_point_link_->begin();
      c_iter != cartesian_point_link_->end(); c_iter++)
  {
    ROS_DEBUG_STREAM("C_ID: " << c_iter->first << "[P_ID: " << c_iter->second.links_.id_previous << " -> N_ID: "
                      << c_iter->second.links_.id_next << "](Joints: " << c_iter->second.joints_.size() << ')');
  }
}

int PlanningGraph::recalculateJointSolutionsVertexMap(std::map<TrajectoryPt::ID, JointGraph::vertex_descriptor> &joint_map)
{
  std::pair<VertexIterator, VertexIterator> vi = vertices(dg_);

  for (VertexIterator vert_iter = vi.first; vert_iter != vi.second; ++vert_iter)
  {
    JointGraph::vertex_descriptor jv = *vert_iter;
    joint_map[dg_[jv].id] = jv;
  }
}

// TODO: optionally output this to a .DOT file (viewable in GraphVIZ or comparable)
void PlanningGraph::printGraph()
{
  ROS_DEBUG_STREAM("\n\nPRINTING GRAPH\n\n");
  std::stringstream ss;
  ss << "GRAPH VERTICES (" << num_vertices(dg_) << "): ";
  ROS_DEBUG_STREAM(ss.str());
  std::pair<VertexIterator, VertexIterator> vi = vertices(dg_);
  ROS_DEBUG_STREAM("Graph OutEdges:");
  for (VertexIterator vert_iter = vi.first; vert_iter != vi.second; ++vert_iter)
  {
    JointGraph::vertex_descriptor jv = *vert_iter;
    ss.str("");
    ss << "Vertex: (" << jv << ")";
    ss << dg_[jv].id;
    std::pair<OutEdgeIterator, OutEdgeIterator> out_ei = out_edges(jv, dg_);
    ss << " -> {";
    for (OutEdgeIterator out_edge = out_ei.first; out_edge != out_ei.second; ++out_edge)
    {
      JointGraph::edge_descriptor e = *out_edge;
      ss << "[" << dg_[e].joint_end << "] , ";
    }
    ss << "}";
    ROS_DEBUG_STREAM(ss.str());
  }

  ROS_DEBUG_STREAM("Graph InEdges:");
  for (VertexIterator vert_iter = vi.first; vert_iter != vi.second; ++vert_iter)
  {
    JointGraph::vertex_descriptor jv = *vert_iter;

    std::pair<InEdgeIterator, InEdgeIterator> in_ei = in_edges(jv, dg_);
    ss.str("");
    ss << "{";
    for (InEdgeIterator in_edge = in_ei.first; in_edge != in_ei.second; ++in_edge)
    {
      JointGraph::edge_descriptor e = *in_edge;
      ss << source(e, dg_) << "[" << dg_[e].joint_start << "], ";
    }
    ss << "} -> ";
    ss << "Vertex (" << jv << "): " ;//<<  dg_[jv].id;
    ROS_DEBUG_STREAM(ss.str());
  }

  ROS_DEBUG_STREAM("GRAPH EDGES (" << num_edges(dg_) << "): ");
  //Tried to make this section more clear, instead of using tie, keeping all
  //the original types so it's more clear what is going on
  std::pair<EdgeIterator, EdgeIterator> ei = edges(dg_);
  for (EdgeIterator edge_iter = ei.first; edge_iter != ei.second; ++edge_iter)
  {
    JointGraph::vertex_descriptor jv = source(*edge_iter, dg_);
    JointGraph::edge_descriptor e = *out_edges(jv, dg_).first;

    JointGraph::edge_descriptor e2 = *edge_iter;
    ROS_DEBUG_STREAM("(" << source(*edge_iter, dg_) << ", " << target(*edge_iter, dg_) << "): cost: " << dg_[e2].transition_cost);
  }
  ROS_DEBUG_STREAM("\n\nEND PRINTING GRAPH\n\n");
}

bool PlanningGraph::calculateJointSolutions()
{
  if (joint_solutions_map_.size() > 0)
  {
    // existing joint solutions... clear the list?
    ROS_WARN_STREAM("existing joint solutions found, clearing map");
    joint_solutions_map_.clear();
  }

  // for each TrajectoryPt, get the available joint solutions
  for (std::map<TrajectoryPt::ID, CartesianPointInformation>::iterator trajectory_iter = cartesian_point_link_->begin();
      trajectory_iter != cartesian_point_link_->end(); trajectory_iter++)
  {
    // TODO: copy this block to a function that can be used by add and modify
    /*************************/
    std::list<TrajectoryPt::ID> *traj_solutions = new std::list<TrajectoryPt::ID>();
    std::vector<std::vector<double> > joint_poses;
    trajectory_iter->second.source_trajectory_.get()->getJointPoses(*robot_model_, joint_poses);

    TrajectoryPt::ID tempID = trajectory_iter->first;
    ROS_INFO_STREAM("CartID: " << tempID << " JointPoses count: " << joint_poses.size());

    if (joint_poses.size() == 0)
    {
      ROS_WARN_STREAM("no joint solution for this point... potential discontinuity in the graph");
    }
    else
    {
      for (std::vector<std::vector<double> >::iterator joint_pose_iter = joint_poses.begin();
          joint_pose_iter != joint_poses.end(); joint_pose_iter++)
      {
        //get UUID from JointTrajPt (convert from std::vector<double>)
        JointTrajectoryPt *new_pt = new JointTrajectoryPt(*joint_pose_iter);
        traj_solutions->push_back(new_pt->getID());
        joint_solutions_map_[new_pt->getID()] = *new_pt;
      }
    }
    trajectory_iter->second.joints_ = *traj_solutions;
    /*************************/
  }

  return true;
}

bool PlanningGraph::calculateAllEdgeWeights(std::list<JointEdge> &edges)
{
  if (cartesian_point_link_->size() == 0)
  {
    // no linkings of cartesian points
    ROS_ERROR_STREAM("no trajectory point links defined");
    return false;
  }
  else
  {
    ROS_DEBUG_STREAM("Found %i " << cartesian_point_link_->size() << " trajectory point links");
  }

  if (joint_solutions_map_.size() == 0)
  {
    // no joint solutions to calculate transitions for
    ROS_ERROR_STREAM("no joint solutions available");
    return false;
  }
  else
  {
    ROS_INFO_STREAM("Found " << joint_solutions_map_.size() << " joint solutions available");
  }

  for (std::map<TrajectoryPt::ID, CartesianPointInformation>::iterator cart_link_iter = cartesian_point_link_->begin();
      cart_link_iter != cartesian_point_link_->end(); cart_link_iter++)
  {
    TrajectoryPt::ID start_cart_id = cart_link_iter->first; // should be equal to cart_link_iter->second.links_.id
    TrajectoryPt::ID end_cart_id = cart_link_iter->second.links_.id_next;

    if(!end_cart_id.is_nil())
    {
      std::list<TrajectoryPt::ID> start_joint_ids = cart_link_iter->second.joints_;
      std::list<TrajectoryPt::ID> end_joint_ids = (*cartesian_point_link_)[end_cart_id].joints_;

      if(!calculateEdgeWeights(start_joint_ids, end_joint_ids, edges))
      {
        ROS_WARN_STREAM("One or more joints lists in the cartesian point link is empty ID:" << start_cart_id << 
                        "[start ids:" << start_joint_ids.size() << "], ID:" << end_cart_id << "[end ids:" << end_joint_ids.size() << ']');
      }
    }
    else
    {
      // not going to try to calculate from point to nil
      ROS_INFO_STREAM("Not calculating edge weights to nil ID");
    }
  }

  return !edges.empty();
}

bool PlanningGraph::calculateEdgeWeights(const std::list<TrajectoryPt::ID> &start_joints,const std::list<TrajectoryPt::ID> &end_joints, std::list<JointEdge> &edge_results)
{
  if(start_joints.empty() || end_joints.empty())
  {
    ROS_WARN_STREAM("One or more joints lists is empty, Start Joints: " << start_joints.size() << " End Joints: " << end_joints.size());
    return false;
  }

  bool has_valid_transition = false;

  // calculate edges for previous vertices to this set of vertices
  for (std::list<TrajectoryPt::ID>::const_iterator previous_joint_iter = start_joints.begin();
      previous_joint_iter != start_joints.end(); previous_joint_iter++)
  {
    for (std::list<TrajectoryPt::ID>::const_iterator next_joint_iter = end_joints.begin();
        next_joint_iter != end_joints.end(); next_joint_iter++)
    {
      double transition_cost;
      const JointTrajectoryPt& start_joint = joint_solutions_map_[*previous_joint_iter];
      const JointTrajectoryPt& end_joint = joint_solutions_map_[*next_joint_iter];

      transition_cost = linearWeight(start_joint, end_joint);

      if (transition_cost >= MAX_EXCEEDED_PENALTY)
      {
        continue;
      }

      has_valid_transition = true;

      JointEdge edge;
      edge.joint_start = *previous_joint_iter;
      edge.joint_end = *next_joint_iter;
      edge.transition_cost = transition_cost;

      edge_results.push_back(edge);
    }
  }

  return has_valid_transition;
}

bool PlanningGraph::populateGraphVertices()
{
  if (joint_solutions_map_.size() == 0)
  {
    // no joints (vertices)
    ROS_ERROR_STREAM("no joint solutions defined, thus no graph vertices");
    return false;
  }

  for (std::map<TrajectoryPt::ID, JointTrajectoryPt>::iterator joint_iter = joint_solutions_map_.begin();
      joint_iter != joint_solutions_map_.end(); joint_iter++)
  {
    JointGraph::vertex_descriptor v = boost::add_vertex(dg_);
    dg_[v].id = joint_iter->second.getID();
  }

  return true;
}

bool PlanningGraph::populateGraphEdges(const std::list<JointEdge> &edges)
{
  if (edges.size() == 0)
  {
    // no edges
    ROS_ERROR_STREAM("no graph edges defined");
    return false;
  }

  std::map<TrajectoryPt::ID, JointGraph::vertex_descriptor> joint_vertex_map;
  int num_joints = recalculateJointSolutionsVertexMap(joint_vertex_map);

  for (std::list<JointEdge>::const_iterator edge_iter = edges.begin(); edge_iter != edges.end(); edge_iter++)
  {
    JointGraph::edge_descriptor e;
    bool b;
    // add graph links for structure
    boost::tie(e, b) = boost::add_edge(joint_vertex_map[edge_iter->joint_start],
                                       joint_vertex_map[edge_iter->joint_end], dg_);
    // populate edge fields
    dg_[e].transition_cost = edge_iter->transition_cost;
    dg_[e].joint_start = edge_iter->joint_start;
    dg_[e].joint_end = edge_iter->joint_end;
  }

  return true;
}

double PlanningGraph::linearWeight(const JointTrajectoryPt& start, const JointTrajectoryPt& end) const
{
  std::vector<std::vector<double> > joint_poses_start;
  start.getJointPoses(*robot_model_, joint_poses_start);

  std::vector<std::vector<double> > joint_poses_end;
  end.getJointPoses(*robot_model_, joint_poses_end);

  // each should only return one
  if (joint_poses_start.size() == 1 && joint_poses_end.size() == 1)
  {
    const std::vector<double>& start_vector = joint_poses_start[0];
    const std::vector<double>& end_vector = joint_poses_end[0];
    
    if (start_vector.size() == end_vector.size())
    {
      double vector_diff = 0;
      double joint_diff = 0;
      for (int i = 0; i < start_vector.size(); i++)
      {
        joint_diff = std::abs(end_vector[i] - start_vector[i]);
        if(joint_diff > MAX_JOINT_DIFF)
        {
          return MAX_EXCEEDED_PENALTY;
        }
        else
        {
          vector_diff += joint_diff ;
        }
      }
      return vector_diff;
    }
    else
    {
      ROS_WARN_STREAM("unequal joint pose vector lengths: " << start_vector.size() << "!=" << end_vector.size());

      return std::numeric_limits<double>::max();
    }
  }
  else
  {
    ROS_WARN_STREAM("invalid joint pose(s) found");
    return std::numeric_limits<double>::max();
  }
}
} /* namespace descartes_planner */
