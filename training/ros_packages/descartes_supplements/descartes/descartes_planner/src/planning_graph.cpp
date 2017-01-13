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
PlanningGraph::PlanningGraph(RobotModelConstPtr model)
  : robot_model_(std::move(model)), cartesian_point_link_(NULL), custom_cost_function_(NULL)
{
}

PlanningGraph::PlanningGraph(RobotModelConstPtr model, CostFunction cost_function_callback)
  : robot_model_(std::move(model)), cartesian_point_link_(NULL), custom_cost_function_(cost_function_callback)
{
}

PlanningGraph::~PlanningGraph()
{
  clear();
}

void PlanningGraph::clear()
{
  delete cartesian_point_link_;
  dg_.clear();
  joint_solutions_map_.clear();
}

CartesianMap PlanningGraph::getCartesianMap() const
{
  if (!cartesian_point_link_)
  {
    return CartesianMap();
  }
  else
    return *cartesian_point_link_;
}

const JointMap& PlanningGraph::getJointMap() const
{
  return joint_solutions_map_;
}

const JointGraph& PlanningGraph::getGraph() const
{
  return dg_;
}

descartes_core::RobotModelConstPtr PlanningGraph::getRobotModel()
{
  return robot_model_;
}

bool PlanningGraph::insertGraph(const std::vector<TrajectoryPtPtr>* points)
{
  // validate input
  if (!points)
  {
    // one or both are null
    ROS_ERROR_STREAM("points == null. Cannot initialize graph with null list.");
    return false;
  }
  if (points->size() < 2)
  {
    ROS_ERROR_STREAM(__FUNCTION__ << ": must provide at least 2 input trajectory points.");
    return false;
  }

  // Reset any previous graph data
  clear();

  // Start new map
  cartesian_point_link_ = new std::map<TrajectoryPt::ID, CartesianPointInformation>();

  // DEBUG
  // printMaps();
  TrajectoryPt::ID previous_id = descartes_core::TrajectoryID::make_nil();

  // input is valid, copy to local maps that will be maintained by the planning graph
  for (auto point_iter = points->begin(); point_iter != points->end(); ++point_iter)
  {
    (*cartesian_point_link_)[point_iter->get()->getID()].source_trajectory_ = (*point_iter);
    CartesianPointRelationship point_link = CartesianPointRelationship();
    point_link.id = point_iter->get()->getID();
    point_link.id_next = descartes_core::TrajectoryID::make_nil();      // default to nil UUID
    point_link.id_previous = descartes_core::TrajectoryID::make_nil();  // default to nil UUID

    // if the previous_id exists, set it's next_id to the new id
    if (cartesian_point_link_->find(previous_id) != cartesian_point_link_->end())
    {
      (*cartesian_point_link_)[previous_id].links_.id_next = point_link.id;

      ROS_DEBUG_STREAM("PreviousID[" << previous_id << "].links_.id_next = " << point_link.id);
    }

    // set the new current point link
    point_link.id_previous = previous_id;

    // the new one becomes the previous_id
    previous_id = point_link.id;

    // save the point_link structure to the map
    (*cartesian_point_link_)[point_link.id].links_ = point_link;
  }

  // after populating maps above (presumably from cartesian trajectory points), calculate (or query) for all joint
  // trajectories
  std::vector<std::vector<JointTrajectoryPt>> poses;
  if (!calculateJointSolutions(*points, poses))
  {
    ROS_ERROR_STREAM("unable to calculate joint trajectories for input points");
    return false;
  }

  std::vector<JointEdge> edges;
  if (!calculateAllEdgeWeights(poses, edges))
  {
    ROS_ERROR_STREAM("unable to calculate edge weight of joint transitions for joint trajectories");
    return false;
  }

  if (!populateGraphVertices(*points, poses))
  {
    ROS_ERROR_STREAM("unable to populate graph from input points");
    return false;
  }

  // from list of joint trajectories (vertices) and edges (edges), construct the actual graph
  if (!populateGraphEdges(edges))
  {
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
  // if (cartesian_point_link_->find(previous_id) != cartesian_point_link_->end())
  if (!previous_id.is_nil())
  {
    (*cartesian_point_link_)[previous_id].links_.id_next = point_link.id;
  }
  // if not updating the end, update the next to point to the new point
  // if (cartesian_point_link_->find(next_id) != cartesian_point_link_->end())
  if (!next_id.is_nil())
  {
    (*cartesian_point_link_)[next_id].links_.id_previous = point_link.id;
  }

  ROS_DEBUG_STREAM("New ID: " << point_link.id);
  ROS_DEBUG_STREAM("New Next ID: " << point_link.id_next);
  ROS_DEBUG_STREAM("New Previous ID: " << point_link.id_previous);

  VertexMap joint_vertex_map;
  recalculateJointSolutionsVertexMap(joint_vertex_map);

  if (!previous_id.is_nil() && !next_id.is_nil())
  {
    // remove graph edges from each joint at [id_previous] to joint at [id_next]
    std::vector<TrajectoryPt::ID> start_joint_ids = (*cartesian_point_link_)[previous_id].joints_;
    std::vector<TrajectoryPt::ID> end_joint_ids = (*cartesian_point_link_)[next_id].joints_;
    for (std::vector<TrajectoryPt::ID>::iterator start_joint_iter = start_joint_ids.begin();
         start_joint_iter != start_joint_ids.end(); start_joint_iter++)
    {
      for (std::vector<TrajectoryPt::ID>::iterator end_joint_iter = end_joint_ids.begin();
           end_joint_iter != end_joint_ids.end(); end_joint_iter++)
      {
        ROS_DEBUG_STREAM("Removing edge: " << *start_joint_iter << " -> " << *end_joint_iter);
        boost::remove_edge(joint_vertex_map[*start_joint_iter], joint_vertex_map[*end_joint_iter], dg_);
      }
    }
  }

  // get joint poses from new trajectory point (unique id)
  std::vector<std::vector<double>> joint_poses;
  point.get()->getJointPoses(*robot_model_, joint_poses);

  if (joint_poses.size() == 0)
  {
    ROS_WARN_STREAM("no joint solution for this point... potential discontinuity in the graph");
  }
  else
  {
    for (std::vector<std::vector<double>>::iterator joint_pose_iter = joint_poses.begin();
         joint_pose_iter != joint_poses.end(); joint_pose_iter++)
    {
      // get UUID from JointTrajPt (convert from std::vector<double>)
      JointTrajectoryPt new_pt(*joint_pose_iter, point->getTiming());
      // traj_solutions->push_back(new_pt->getID());
      (*cartesian_point_link_)[point->getID()].joints_.push_back(new_pt.getID());

      // insert new vertices into graph
      JointGraph::vertex_descriptor v = boost::add_vertex(dg_);
      dg_[v].id = new_pt.getID();

      joint_solutions_map_[new_pt.getID()] = new_pt;
    }
  }

  // save the list of joint solutions
  std::vector<TrajectoryPt::ID> traj_solutions = (*cartesian_point_link_)[point->getID()].joints_;
  // save the actual trajectory point into the map
  (*cartesian_point_link_)[point->getID()].source_trajectory_ = point;

  ROS_INFO_STREAM("SAVE CART POINT ID[" << point->getID() << "]: "
                                        << (*cartesian_point_link_)[point->getID()].source_trajectory_.get()->getID());

  std::vector<JointEdge> edges;
  // recalculate edges(previous -> this; this -> next)

  if (!previous_id.is_nil())
  {
    std::vector<TrajectoryPt::ID> previous_joint_ids = (*cartesian_point_link_)[previous_id].joints_;
    calculateEdgeWeights(previous_joint_ids, traj_solutions, edges);
  }

  if (!next_id.is_nil())
  {
    std::vector<TrajectoryPt::ID> next_joint_ids = (*cartesian_point_link_)[next_id].joints_;
    calculateEdgeWeights(traj_solutions, next_joint_ids, edges);
  }

  // insert new edges
  populateGraphEdges(edges);

  // DEBUG LOGS
  //  printGraph();
  //  printMaps();

  // CartesianMap test_map = getCartesianMap();

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
  std::vector<TrajectoryPt::ID> start_joint_ids = (*cartesian_point_link_)[modify_id].joints_;

  ROS_INFO_STREAM("start_joint_ids.size = " << start_joint_ids.size());

  std::vector<JointGraph::edge_descriptor> to_remove_edges;
  std::vector<JointGraph::vertex_descriptor> to_remove_vertices;
  // remove edges
  for (std::vector<TrajectoryPt::ID>::iterator start_joint_iter = start_joint_ids.begin();
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
  for (std::vector<JointGraph::edge_descriptor>::iterator e_iter = to_remove_edges.begin();
       e_iter != to_remove_edges.end(); e_iter++)
  {
    ROS_DEBUG_STREAM("REMOVE EDGE: " << dg_[*e_iter].joint_start << " -> " << dg_[*e_iter].joint_end);
  boost:
    remove_edge(*e_iter, dg_);
    // printGraph();
  }
  std::sort(to_remove_vertices.begin(), to_remove_vertices.end());
  std::reverse(to_remove_vertices.begin(), to_remove_vertices.end());
  for (std::vector<JointGraph::vertex_descriptor>::iterator v_iter = to_remove_vertices.begin();
       v_iter != to_remove_vertices.end(); v_iter++)
  {
    // remove the graph vertex and joint point
    // NOTE: cannot print jv as int when using listS
    ROS_DEBUG_STREAM("REMOVE VERTEX: " << *v_iter);
    boost::remove_vertex(*v_iter, dg_);
    //    printGraph();
  }
  (*cartesian_point_link_)[modify_id].joints_.clear();

  // get new joint points for this cartesian
  std::vector<std::vector<double>> joint_poses;
  point.get()->getJointPoses(*robot_model_, joint_poses);

  if (joint_poses.size() == 0)
  {
    ROS_WARN_STREAM("no joint solution for this point... potential discontinuity in the graph");
  }
  else
  {
    for (std::vector<std::vector<double>>::iterator joint_pose_iter = joint_poses.begin();
         joint_pose_iter != joint_poses.end(); joint_pose_iter++)
    {
      // get UUID from JointTrajPt (convert from std::vector<double>)
      JointTrajectoryPt new_pt(*joint_pose_iter, point->getTiming());
      (*cartesian_point_link_)[modify_id].joints_.push_back(new_pt.getID());

      // insert new vertices into graph
      JointGraph::vertex_descriptor v = boost::add_vertex(dg_);
      dg_[v].id = new_pt.getID();

      joint_solutions_map_[new_pt.getID()] = new_pt;
      ROS_INFO_STREAM("Added New Joint: " << v);
    }
  }

  std::vector<TrajectoryPt::ID> traj_solutions = (*cartesian_point_link_)[modify_id].joints_;
  (*cartesian_point_link_)[modify_id].source_trajectory_ = point;
  // don't need to modify links

  // recalculate edges(previous -> this; this -> next)
  TrajectoryPt::ID previous_cart_id =
      (*cartesian_point_link_)[modify_id].links_.id_previous;  // should be equal to cart_link_iter->second.links_.id
  TrajectoryPt::ID next_cart_id = (*cartesian_point_link_)[modify_id].links_.id_next;

  std::vector<JointEdge> edges;
  // recalculate edges(previous -> this; this -> next)

  if (!previous_cart_id.is_nil())
  {
    std::vector<TrajectoryPt::ID> previous_joint_ids = (*cartesian_point_link_)[previous_cart_id].joints_;
    ROS_DEBUG_STREAM("Calculating previous -> new weights: " << previous_joint_ids.size() << " -> "
                                                             << traj_solutions.size());
    calculateEdgeWeights(previous_joint_ids, traj_solutions, edges);
  }

  if (!next_cart_id.is_nil())
  {
    std::vector<TrajectoryPt::ID> next_joint_ids = (*cartesian_point_link_)[next_cart_id].joints_;
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
  std::vector<TrajectoryPt::ID> start_joint_ids = (*cartesian_point_link_)[delete_id].joints_;

  ROS_INFO_STREAM("Attempting to delete edges from " << start_joint_ids.size() << " vertices");

  std::map<TrajectoryPt::ID, JointGraph::vertex_descriptor> joint_vertex_map;
  int num_joints = recalculateJointSolutionsVertexMap(joint_vertex_map);

  std::vector<JointGraph::edge_descriptor> to_remove_edges;
  std::vector<JointGraph::vertex_descriptor> to_remove_vertices;
  // remove edges
  for (std::vector<TrajectoryPt::ID>::iterator start_joint_iter = start_joint_ids.begin();
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

  for (std::vector<JointGraph::edge_descriptor>::iterator e_iter = to_remove_edges.begin();
       e_iter != to_remove_edges.end(); e_iter++)
  {
  boost:
    remove_edge(*e_iter, dg_);
  }

  std::sort(to_remove_vertices.begin(), to_remove_vertices.end());
  std::reverse(to_remove_vertices.begin(), to_remove_vertices.end());
  for (std::vector<JointGraph::vertex_descriptor>::iterator v_iter = to_remove_vertices.begin();
       v_iter != to_remove_vertices.end(); v_iter++)
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

  if (!previous_id.is_nil())
  {
    // change previous_cartesian link to next_id
    (*cartesian_point_link_)[previous_id].links_.id_next = next_id;
  }
  if (!next_id.is_nil())
  {
    // change next_cartesian link to previous_id
    (*cartesian_point_link_)[next_id].links_.id_previous = previous_id;
  }

  std::vector<JointEdge> edges;
  // recalculate edges(previous -> this; this -> next)

  if (!previous_id.is_nil() && !next_id.is_nil())
  {
    std::vector<TrajectoryPt::ID> previous_joint_ids = (*cartesian_point_link_)[previous_id].joints_;
    std::vector<TrajectoryPt::ID> next_joint_ids = (*cartesian_point_link_)[next_id].joints_;
    calculateEdgeWeights(previous_joint_ids, next_joint_ids, edges);
  }

  // insert new edges
  populateGraphEdges(edges);

  // DEBUG LOGS
  //  printGraph();
  //  printMaps();

  return true;
}

bool PlanningGraph::findStartVertices(std::vector<JointGraph::vertex_descriptor>& start_points) const
{
  // Create local id->vertex map
  VertexMap joint_vertex_map;
  recalculateJointSolutionsVertexMap(joint_vertex_map);

  // Find the TrajectoryPt ID of the first point specified by the user
  TrajectoryPt::ID cart_id = descartes_core::TrajectoryID::make_nil();

  for (auto c_iter = cartesian_point_link_->begin(); c_iter != cartesian_point_link_->end(); ++c_iter)
  {
    if (c_iter->second.links_.id_previous.is_nil())
    {
      cart_id = c_iter->first;
      break;
    }
  }

  // Test for error case
  if (cart_id.is_nil())
  {
    // Users can insert points with any given previous and next point using the addTrajectory
    // function.
    ROS_ERROR("Could not locate TrajectoryPt with nil previous point. Graph may be cyclic.");
    return false;
  }

  // Iterate through the JointSolutions for the last point
  const std::vector<descartes_core::TrajectoryPt::ID>& ids = cartesian_point_link_->at(cart_id).joints_;

  for (const auto& id : ids)
  {
    JointGraph::vertex_descriptor v = joint_vertex_map.at(id);
    start_points.push_back(v);
  }

  return !start_points.empty();
}

bool PlanningGraph::findEndVertices(std::vector<JointGraph::vertex_descriptor>& end_points) const
{
  // Create local id->vertex map
  VertexMap joint_vertex_map;
  recalculateJointSolutionsVertexMap(joint_vertex_map);

  // Find the TrajectoryPt ID of the last point specified by the user
  TrajectoryPt::ID cart_id = descartes_core::TrajectoryID::make_nil();

  for (auto c_iter = cartesian_point_link_->begin(); c_iter != cartesian_point_link_->end(); ++c_iter)
  {
    if (c_iter->second.links_.id_next.is_nil())
    {
      cart_id = c_iter->first;
      break;
    }
  }

  // Test for error case
  if (cart_id.is_nil())
  {
    // Users can insert points with any given previous and next point using the addTrajectory
    // function.
    ROS_ERROR("Could not locate TrajectoryPt with nil next point. Graph may be cyclic.");
    return false;
  }

  // Iterate through the JointSolutions for the last point
  const std::vector<descartes_core::TrajectoryPt::ID>& ids = cartesian_point_link_->at(cart_id).joints_;

  for (const auto& id : ids)
  {
    JointGraph::vertex_descriptor v = joint_vertex_map.at(id);
    end_points.push_back(v);
  }

  return !end_points.empty();
}

bool PlanningGraph::getShortestPath(double& cost, std::list<JointTrajectoryPt>& path)
{
  // Enumerate the start & end vertice descriptors
  std::vector<JointGraph::vertex_descriptor> start_points;
  findStartVertices(start_points);
  std::vector<JointGraph::vertex_descriptor> end_points;
  findEndVertices(end_points);

  // Insert 'virtual' root joint, so that we can search the entire graph
  // even with multiple start vertices
  JointGraph::vertex_descriptor virtual_vertex = boost::add_vertex(dg_);
  for (const auto& pt : start_points)
  {
    auto e = boost::add_edge(virtual_vertex, pt, dg_);
    dg_[e.first].transition_cost = 0.0;
  }

  // Generate mapping of vertex descriptors to Trajectory Point IDs
  size_t num_vert = boost::num_vertices(dg_);
  std::vector<TrajectoryPt::ID> vertex_index_map(num_vert);
  std::pair<VertexIterator, VertexIterator> vi = boost::vertices(dg_);
  int i = 0;
  for (VertexIterator vert_iter = vi.first; vert_iter != vi.second; ++vert_iter)
  {
    JointGraph::vertex_descriptor jv = *vert_iter;
    vertex_index_map[i++] = dg_[jv].id;
  }

  // Default the case to a very large number incase there is no valid path
  // through the graph
  cost = std::numeric_limits<double>::max();

  // Remember which lowest cost end point should be used to create the solution path
  JointGraph::vertex_descriptor cheapest_end_point;

  // initialize vectors to be used by dijkstra
  std::vector<JointGraph::vertex_descriptor> predecessors(num_vert);
  std::vector<double> weights(num_vert, std::numeric_limits<double>::max());

  dijkstra_shortest_paths(
      dg_,
      virtual_vertex,  // start from our fake point
      weight_map(get(&JointEdge::transition_cost, dg_))
          .distance_map(boost::make_iterator_property_map(weights.begin(), get(boost::vertex_index, dg_)))
          .predecessor_map(&predecessors[0]));

  // Search the ending points for a minimum point
  for (auto end = end_points.begin(); end != end_points.end(); ++end)
  {
    double weight = weights[*end];
    // if the weight of this path is less than the previous best, replace the return path
    if (weight < cost)
    {
      cost = weight;
      cheapest_end_point = *end;
    }
  }

  // Create the solution path with the lowest-cost endpoint
  path.clear();
  auto current = cheapest_end_point;
  // Starting from the destination point step through the predecessor map until the source point is reached.
  while (current != virtual_vertex)
  {
    path.push_front(joint_solutions_map_[vertex_index_map[current]]);
    current = predecessors[current];
  }

  // Undo the virtual joint
  boost::clear_vertex(virtual_vertex, dg_);
  boost::remove_vertex(virtual_vertex, dg_);

  if (cost < std::numeric_limits<double>::max())
  {
    ROS_INFO_STREAM("Descartes found path with total cost: " << cost);
    return true;
  }
  else
  {
    ROS_ERROR_STREAM("Descartes unable to find path");
    return false;
  }
}

void PlanningGraph::printMaps()
{
  ROS_DEBUG_STREAM("Number of points: " << cartesian_point_link_->size());

  for (std::map<TrajectoryPt::ID, CartesianPointInformation>::iterator c_iter = cartesian_point_link_->begin();
       c_iter != cartesian_point_link_->end(); c_iter++)
  {
    ROS_DEBUG_STREAM("C_ID: " << c_iter->first << "[P_ID: " << c_iter->second.links_.id_previous << " -> N_ID: "
                              << c_iter->second.links_.id_next << "](Joints: " << c_iter->second.joints_.size() << ')');
  }
}

int PlanningGraph::recalculateJointSolutionsVertexMap(VertexMap& joint_map) const
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
    ss << "Vertex (" << jv << "): ";  //<<  dg_[jv].id;
    ROS_DEBUG_STREAM(ss.str());
  }

  ROS_DEBUG_STREAM("GRAPH EDGES (" << num_edges(dg_) << "): ");
  // Tried to make this section more clear, instead of using tie, keeping all
  // the original types so it's more clear what is going on
  std::pair<EdgeIterator, EdgeIterator> ei = edges(dg_);
  for (EdgeIterator edge_iter = ei.first; edge_iter != ei.second; ++edge_iter)
  {
    JointGraph::vertex_descriptor jv = source(*edge_iter, dg_);
    JointGraph::edge_descriptor e = *out_edges(jv, dg_).first;

    JointGraph::edge_descriptor e2 = *edge_iter;
    ROS_DEBUG_STREAM("(" << source(*edge_iter, dg_) << ", " << target(*edge_iter, dg_)
                         << "): cost: " << dg_[e2].transition_cost);
  }
  ROS_DEBUG_STREAM("\n\nEND PRINTING GRAPH\n\n");
}

bool PlanningGraph::calculateJointSolutions(const std::vector<TrajectoryPtPtr>& points,
                                            std::vector<std::vector<JointTrajectoryPt>>& poses)
{
  poses.resize(points.size());

  for (std::size_t i = 0; i < points.size(); ++i)
  {
    std::vector<std::vector<double>> joint_poses;
    points[i]->getJointPoses(*robot_model_, joint_poses);

    if (joint_poses.empty())
    {
      ROS_ERROR_STREAM(__FUNCTION__ << ": IK failed for input trajectory point with ID = " << points[i]->getID());
      return false;
    }

    poses[i].reserve(joint_poses.size());
    for (auto& sol : joint_poses)
    {
      poses[i].emplace_back(std::move(sol), points[i]->getTiming());
    }
  }

  return true;
}

bool PlanningGraph::calculateAllEdgeWeights(const std::vector<std::vector<JointTrajectoryPt>>& poses,
                                            std::vector<JointEdge>& edges)
{
  // We check that the size of input traj is at least 2 at the start of insertGraph()
  // iterate over each pair of points
  for (std::size_t i = 1; i < poses.size(); ++i)
  {
    const std::vector<JointTrajectoryPt>& from = poses[i - 1];
    const std::vector<JointTrajectoryPt>& to = poses[i];

    if (!calculateEdgeWeights(from, to, edges))
    {
      ROS_ERROR_STREAM(__FUNCTION__ << ": unable to calculate any valid transitions between inputs " << (i - 1)
                                    << " and " << i);
      return false;
    }
  }

  return !edges.empty();
}

bool PlanningGraph::calculateEdgeWeights(const std::vector<TrajectoryPt::ID>& start_joints,
                                         const std::vector<TrajectoryPt::ID>& end_joints,
                                         std::vector<JointEdge>& edge_results)
{
  if (start_joints.empty() || end_joints.empty())
  {
    ROS_WARN_STREAM("One or more joints lists is empty, Start Joints: " << start_joints.size()
                                                                        << " End Joints: " << end_joints.size());
    return false;
  }

  bool has_valid_transition = false;

  // calculate edges for previous vertices to this set of vertices
  for (auto previous_joint_iter = start_joints.begin(); previous_joint_iter != start_joints.end();
       ++previous_joint_iter)
  {
    // Look up the start_joint once per iteration of this loop
    const JointTrajectoryPt& start_joint = joint_solutions_map_[*previous_joint_iter];

    // Loop over the ending points -> look at each combination of start/end points
    for (auto next_joint_iter = end_joints.begin(); next_joint_iter != end_joints.end(); ++next_joint_iter)
    {
      const JointTrajectoryPt& end_joint = joint_solutions_map_[*next_joint_iter];

      EdgeWeightResult edge_result = edgeWeight(start_joint, end_joint);

      // If the edge weight calculation returns false, do not create an edge
      if (!edge_result.first)
      {
        continue;
      }

      // If one edge exists, there is a valid transition from the previous TrajectoryPt
      // to the next
      has_valid_transition = true;

      JointEdge edge;
      edge.joint_start = *previous_joint_iter;
      edge.joint_end = *next_joint_iter;
      edge.transition_cost = edge_result.second;
      edge_results.push_back(edge);
    }
  }

  return has_valid_transition;
}

bool PlanningGraph::calculateEdgeWeights(const std::vector<JointTrajectoryPt>& start_joints,
                                         const std::vector<JointTrajectoryPt>& end_joints,
                                         std::vector<JointEdge>& edge_results) const
{
  if (start_joints.empty() || end_joints.empty())
  {
    ROS_WARN_STREAM("One or more joints lists is empty, Start Joints: " << start_joints.size()
                                                                        << " End Joints: " << end_joints.size());
    return false;
  }

  auto start_size = edge_results.size();

  // calculate edges for previous vertices to this set of vertices
  for (const auto& start_joint : start_joints)
  {
    // Loop over the ending points -> look at each combination of start/end points
    for (const auto& end_joint : end_joints)
    {
      // Calculate edge cost
      EdgeWeightResult edge_result = edgeWeight(start_joint, end_joint);

      // If the edge weight calculation returns false, do not create an edge
      if (!edge_result.first)
        continue;

      JointEdge edge;
      edge.joint_start = start_joint.getID();
      edge.joint_end = end_joint.getID();
      edge.transition_cost = edge_result.second;
      edge_results.push_back(edge);
    }
  }

  return edge_results.size() > start_size;
}

bool PlanningGraph::populateGraphVertices(const std::vector<TrajectoryPtPtr>& points,
                                          std::vector<std::vector<JointTrajectoryPt>>& poses)
{
  if (points.size() != poses.size())
  {
    ROS_ERROR_STREAM(__FUNCTION__ << ": user trajectory and joint solutions should have same length");
    return false;
  }

  if (!joint_solutions_map_.empty())
  {
    ROS_WARN_STREAM(__FUNCTION__ << ": Clearing joint solutions map.");
    joint_solutions_map_.clear();
  }

  // Need to update the boost graph, the joint solutions map, and the cartesian point links
  for (std::size_t i = 0; i < points.size(); ++i)
  {
    // Copy joint solution IDs into cartesian_point_links
    std::vector<TrajectoryPt::ID> ids;
    ids.reserve(poses[i].size());

    std::transform(poses[i].begin(), poses[i].end(), std::back_inserter(ids), [](const JointTrajectoryPt& pt)
                   {
                     return pt.getID();
                   });

    auto& entry = cartesian_point_link_->at(points[i]->getID());
    entry.joints_ = std::move(ids);

    // For each joint solution, add it to joint sol map and the graph
    for (std::size_t j = 0; j < poses[i].size(); ++j)
    {
      // insert into graph
      auto id = poses[i][j].getID();
      auto vertex = boost::add_vertex(dg_);
      dg_[vertex].id = id;
      // insert into sol map
      joint_solutions_map_.emplace(id, std::move(poses[i][j]));
    }
  }

  return true;
}

bool PlanningGraph::populateGraphEdges(const std::vector<JointEdge>& edges)
{
  if (edges.size() == 0)
  {
    // no edges
    ROS_ERROR_STREAM("no graph edges defined");
    return false;
  }

  VertexMap joint_vertex_map;
  recalculateJointSolutionsVertexMap(joint_vertex_map);

  for (auto edge_iter = edges.begin(); edge_iter != edges.end(); ++edge_iter)
  {
    JointGraph::edge_descriptor e;
    bool b;
    // add graph links for structure
    boost::tie(e, b) =
        boost::add_edge(joint_vertex_map[edge_iter->joint_start], joint_vertex_map[edge_iter->joint_end], dg_);
    // populate edge fields
    dg_[e].transition_cost = edge_iter->transition_cost;
    dg_[e].joint_start = edge_iter->joint_start;
    dg_[e].joint_end = edge_iter->joint_end;
  }

  return true;
}

PlanningGraph::EdgeWeightResult PlanningGraph::edgeWeight(const JointTrajectoryPt& start,
                                                          const JointTrajectoryPt& end) const
{
  EdgeWeightResult result;
  result.first = false;

  const std::vector<double>& start_vector = start.nominal();
  const std::vector<double>& end_vector = end.nominal();
  if (start_vector.size() == end_vector.size())
  {
    // Check to see if time is specified and if so, check to see if the
    // joint motion is possible in the window provided
    if (end.getTiming().isSpecified() && !robot_model_->isValidMove(start_vector, end_vector, end.getTiming().upper))
    {
      return result;
    }

    if (custom_cost_function_)
    {
      result.second = custom_cost_function_(start_vector, end_vector);
    }
    else
    {
      double vector_diff = 0;
      for (unsigned i = 0; i < start_vector.size(); i++)
      {
        double joint_diff = std::abs(end_vector[i] - start_vector[i]);
        vector_diff += joint_diff;
      }
      result.second = vector_diff;
    }

    result.first = true;
    return result;
  }
  else
  {
    ROS_WARN_STREAM("unequal joint pose vector lengths: " << start_vector.size() << " != " << end_vector.size());
  }

  return result;
}

} /* namespace descartes_planner */
