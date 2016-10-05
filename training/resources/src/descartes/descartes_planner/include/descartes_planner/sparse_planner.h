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
 * sparse_planner.h
 *
 *  Created on: Dec 17, 2014
 *      Author: ros developer 
 */

#ifndef SPARSE_PLANNER_H_
#define SPARSE_PLANNER_H_

#include <descartes_core/path_planner_base.h>
#include <descartes_planner/planning_graph.h>
#include <tuple>

namespace descartes_planner
{

class SparsePlanner: public descartes_core::PathPlannerBase
{

public:
  typedef std::vector<std::tuple<int,descartes_core::TrajectoryPtPtr,descartes_trajectory::JointTrajectoryPt> > SolutionArray;
public:
  SparsePlanner(descartes_core::RobotModelConstPtr model,double sampling = 0.1f);
  SparsePlanner();
  virtual ~SparsePlanner();

  virtual bool initialize(descartes_core::RobotModelConstPtr model);
  virtual bool setConfig(const descartes_core::PlannerConfig& config);
  virtual void getConfig(descartes_core::PlannerConfig& config) const;
  virtual bool planPath(const std::vector<descartes_core::TrajectoryPtPtr>& traj);
  virtual bool addAfter(const descartes_core::TrajectoryPt::ID& ref_id,descartes_core::TrajectoryPtPtr cp);
  virtual bool addBefore(const descartes_core::TrajectoryPt::ID& ref_id,descartes_core::TrajectoryPtPtr cp);
  virtual bool modify(const descartes_core::TrajectoryPt::ID& ref_id,descartes_core::TrajectoryPtPtr cp);
  virtual bool remove(const descartes_core::TrajectoryPt::ID& ref_id);
  virtual bool getPath(std::vector<descartes_core::TrajectoryPtPtr>& path) const;
  virtual int getErrorCode() const;
  virtual bool getErrorMessage(int error_code, std::string& msg) const;


  void setSampling(double sampling);
  const std::map<descartes_core::TrajectoryPt::ID,descartes_trajectory::JointTrajectoryPt>& getSolution();
  bool getSolutionJointPoint(const descartes_trajectory::CartTrajectoryPt::ID& cart_id,descartes_trajectory::JointTrajectoryPt& j);

protected:

  bool plan();
  bool interpolateJointPose(const std::vector<double>& start,const std::vector<double>& end,
                   double t,std::vector<double>& interp);
  int interpolateSparseTrajectory(const SolutionArray& sparse_solution,int &sparse_index, int &point_pos);
  void sampleTrajectory(double sampling,const std::vector<descartes_core::TrajectoryPtPtr>& dense_trajectory_array,
                        std::vector<descartes_core::TrajectoryPtPtr>& sparse_trajectory_array);

  int getDensePointIndex(const descartes_core::TrajectoryPt::ID& ref_id);
  int getSparsePointIndex(const descartes_core::TrajectoryPt::ID& ref_id);
  int findNearestSparsePointIndex(const descartes_core::TrajectoryPt::ID& ref_id,bool skip_equal = true);
  bool isInSparseTrajectory(const descartes_core::TrajectoryPt::ID& ref_id);
  bool checkJointChanges(const std::vector<double>& s1,
                                        const std::vector<double>& s2, const double& max_change);

  bool getOrderedSparseArray(std::vector<descartes_core::TrajectoryPtPtr>& sparse_array);
  bool getSparseSolutionArray(SolutionArray& sparse_solution_array);

protected:

  enum class InterpolationResult: int
  {
    ERROR = -1,
    REPLAN,
    SUCCESS
  };

  double sampling_;
  int error_code_;
  std::map<int,std::string> error_map_;
  descartes_core::PlannerConfig config_;
  boost::shared_ptr<PlanningGraph> planning_graph_;
  std::vector<descartes_core::TrajectoryPtPtr> cart_points_;
  SolutionArray sparse_solution_array_;
  std::map<descartes_core::TrajectoryPt::ID,descartes_trajectory::JointTrajectoryPt> joint_points_map_;


};

} /* namespace descartes_planner */
#endif /* SPARSE_PLANNER_H_ */
