/*
 * dense_planner.cpp
 *
 *  Created on: Feb 9, 2015
 *      Author: ros developer 
 */

#include <descartes_planner/dense_planner.h>
#include <boost/uuid/uuid_io.hpp> // streaming operators

namespace descartes_planner
{

using namespace descartes_core;
DensePlanner::DensePlanner():
    planning_graph_(),
    error_code_(descartes_core::PlannerError::UNINITIALIZED)
{
 error_map_ = {
                {PlannerError::OK,"OK"},
                {PlannerError::EMPTY_PATH,"No path plan has been generated"},
                {PlannerError::INVALID_ID,"ID is nil or isn't part of the path"},
                {PlannerError::IK_NOT_AVAILABLE,"One or more ik solutions could not be found"},
                {PlannerError::UNINITIALIZED,"Planner has not been initialized with a robot model"},
                {PlannerError::INCOMPLETE_PATH,"Input trajectory and output path point cound differ"}
              };
}

DensePlanner::~DensePlanner()
{

}

bool DensePlanner::initialize(descartes_core::RobotModelConstPtr model)
{
  planning_graph_ = boost::shared_ptr<descartes_planner::PlanningGraph>(
      new descartes_planner::PlanningGraph(std::move(model)));
  error_code_ = descartes_core::PlannerErrors::EMPTY_PATH;
  return true;
}

bool DensePlanner::setConfig(const descartes_core::PlannerConfig& config)
{
  config_ = config;
  config_.clear();
  return true;
}

void DensePlanner::getConfig(descartes_core::PlannerConfig& config) const
{
  config = config_;
}

descartes_core::TrajectoryPt::ID DensePlanner::getPrevious(const descartes_core::TrajectoryPt::ID& ref_id)
{
  descartes_core::TrajectoryPt::ID id;
  auto predicate = [&ref_id](descartes_core::TrajectoryPtPtr p)
    {
      return ref_id == p->getID();
    };

  auto pos = std::find_if(path_.begin()++,path_.end(),predicate);
  if(pos == path_.end() )
  {
    id =  descartes_core::TrajectoryID::make_nil();
  }
  else
  {
    pos--;
    id =  (*pos)->getID();
  }

  return id;
}

bool DensePlanner::updatePath()
{
  std::vector<descartes_core::TrajectoryPtPtr> traj;
  const CartesianMap& cart_map = planning_graph_->getCartesianMap();
  descartes_core::TrajectoryPt::ID first_id = descartes_core::TrajectoryID::make_nil();
  auto predicate = [&first_id](const std::pair<descartes_core::TrajectoryPt::ID,CartesianPointInformation>& p)
    {
      const auto& info = p.second;
      if(info.links_.id_previous == descartes_core::TrajectoryID::make_nil())
      {
        first_id = p.first;
        return true;
      }
      else
      {
        return false;
      }
    };

  // finding first point
  if(cart_map.empty()
      || (std::find_if(cart_map.begin(),cart_map.end(),predicate) == cart_map.end())
      || first_id == descartes_core::TrajectoryID::make_nil())
  {
    error_code_ = descartes_core::PlannerError::INVALID_ID;
    return false;
  }

  // retrieving original trajectory
  traj.resize(cart_map.size());
  descartes_core::TrajectoryPt::ID current_id = first_id;
  for(int i = 0; i < traj.size(); i++)
  {
    if(cart_map.count(current_id) == 0)
    {
      ROS_ERROR_STREAM("Trajectory point "<<current_id<<" was not found in cartesian trajectory.");
      return false;
    }
    const CartesianPointInformation& info  = cart_map.at(current_id);
    traj[i] = info.source_trajectory_;
    current_id = info.links_.id_next;
  }

  // updating planned path
  std::list<descartes_trajectory::JointTrajectoryPt> list;
  double cost;
  if(planning_graph_->getShortestPath(cost,list) )
  {
    if(traj.size() == list.size())
    {
      error_code_ = descartes_core::PlannerError::OK;

      // reassigning ids
      int counter = 0;
      path_.resize(list.size());
      for(auto p : list)
      {
        path_[counter] = descartes_core::TrajectoryPtPtr(new descartes_trajectory::JointTrajectoryPt(p));
        path_[counter]->setID(traj[counter]->getID());
        counter++;
      }
    }
    else
    {
      error_code_ = descartes_core::PlannerError::INCOMPLETE_PATH;
    }
  }
  else
  {
    error_code_ = descartes_core::PlannerError::IK_NOT_AVAILABLE;
  }

  return error_code_ == descartes_core::PlannerError::OK;
}

descartes_core::TrajectoryPt::ID DensePlanner::getNext(const descartes_core::TrajectoryPt::ID& ref_id)
{
  descartes_core::TrajectoryPt::ID id;
  auto predicate = [&ref_id](descartes_core::TrajectoryPtPtr p)
    {
      return ref_id == p->getID();
    };

  auto pos = std::find_if(path_.begin(),path_.end()-2,predicate);
  if(pos == path_.end() )
  {
    id =  descartes_core::TrajectoryID::make_nil();
  }
  else
  {
    pos++;
    id =  (*pos)->getID();
  }
  return id;
}

descartes_core::TrajectoryPtPtr DensePlanner::get(const descartes_core::TrajectoryPt::ID& ref_id)
{
  descartes_core::TrajectoryPtPtr p;
  auto predicate = [&ref_id](descartes_core::TrajectoryPtPtr p)
    {
      return ref_id == p->getID();
    };

  auto pos = std::find_if(path_.begin(),path_.end()-2,predicate);
  if(pos == path_.end() )
  {
    p.reset();
  }
  else
  {
    p  =  *pos;
  }
  return p;
}

bool DensePlanner::planPath(const std::vector<descartes_core::TrajectoryPtPtr>& traj)
{
  if(error_code_ == descartes_core::PlannerError::UNINITIALIZED)
  {
    ROS_ERROR_STREAM("Planner has not been initialized");
    return false;
  }

  double cost;
  path_.clear();
  error_code_ = descartes_core::PlannerError::EMPTY_PATH;

  if(planning_graph_->insertGraph(&traj))
  {
    updatePath();
  }
  else
  {
    error_code_ = descartes_core::PlannerError::IK_NOT_AVAILABLE;
  }

  return descartes_core::PlannerError::OK == error_code_;
}

bool DensePlanner::getPath(std::vector<descartes_core::TrajectoryPtPtr>& path) const
{
  if (path_.empty()) return false;
  
  path.assign(path_.begin(),path_.end());
  return error_code_ == descartes_core::PlannerError::OK;
}

bool DensePlanner::addAfter(const descartes_core::TrajectoryPt::ID& ref_id, descartes_core::TrajectoryPtPtr tp)
{
  if(path_.empty())
  {
    return false;
  }

  descartes_core::TrajectoryPt::ID next_id = getNext(ref_id);
  if(!next_id.is_nil())
  {
    if(planning_graph_->addTrajectory(tp,ref_id,next_id))
    {
      if(updatePath())
      {
        error_code_ = descartes_core::PlannerError::OK;
      }
      else
      {
        return false;
      }
    }
    else
    {
      error_code_ = descartes_core::PlannerErrors::IK_NOT_AVAILABLE;
      return false;
    }
  }
  else
  {
    error_code_ = descartes_core::PlannerError::INVALID_ID;
    return false;
  }

  return true;
}

bool DensePlanner::addBefore(const descartes_core::TrajectoryPt::ID& ref_id, descartes_core::TrajectoryPtPtr tp)
{
  if(path_.empty())
  {
    return false;
  }

  descartes_core::TrajectoryPt::ID prev_id = getPrevious(ref_id);
  if(!prev_id.is_nil())
  {
    if(planning_graph_->addTrajectory(tp,prev_id,ref_id))
    {
      if(updatePath())
      {
        error_code_ = descartes_core::PlannerError::OK;
      }
      else
      {
        return false;
      }
    }
    else
    {
      error_code_ = descartes_core::PlannerErrors::IK_NOT_AVAILABLE;
      return false;
    }
  }
  else
  {
    error_code_ = descartes_core::PlannerError::INVALID_ID;
    return false;
  }

  return true;
}

bool DensePlanner::remove(const descartes_core::TrajectoryPt::ID& ref_id)
{
  if(path_.empty())
  {
    return false;
  }

  descartes_core::TrajectoryPtPtr tp = get(ref_id);
  if(tp)
  {
    tp->setID(ref_id);
    if(planning_graph_->removeTrajectory(tp))
    {
      if(updatePath())
      {
        error_code_ = descartes_core::PlannerError::OK;
      }
      else
      {
        return false;
      }
    }
    else
    {
      error_code_ = descartes_core::PlannerErrors::IK_NOT_AVAILABLE;
      return false;
    }
  }
  else
  {
    error_code_ = descartes_core::PlannerError::INVALID_ID;
    return false;
  }

  return true;
}

bool DensePlanner::modify(const descartes_core::TrajectoryPt::ID& ref_id, descartes_core::TrajectoryPtPtr tp)
{
  if(path_.empty())
  {
    return false;
  }

  if(!ref_id.is_nil())
  {
    tp->setID(ref_id);
    if(planning_graph_->modifyTrajectory(tp))
    {
      if(updatePath())
      {
        error_code_ = descartes_core::PlannerError::OK;
      }
      else
      {
        return false;
      }
    }
    else
    {
      error_code_ = descartes_core::PlannerErrors::IK_NOT_AVAILABLE;
      return false;
    }
  }
  else
  {
    error_code_ = descartes_core::PlannerError::INVALID_ID;
    return false;
  }

  return true;
}

int DensePlanner::getErrorCode() const
{
  return error_code_;
}

bool DensePlanner::getErrorMessage(int error_code, std::string& msg) const
{
  std::map<int,std::string>::const_iterator it = error_map_.find(error_code);

  if(it != error_map_.cend())
  {
    msg = it->second;
  }
  else
  {
    return false;
  }
  return true;
}


} /* namespace descartes_core */
