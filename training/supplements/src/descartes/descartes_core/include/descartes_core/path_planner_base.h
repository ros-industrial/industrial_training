/*
 * path_planner_base.h
 *
 *  Created on: Jan 19, 2015
 *      Author: ros developer 
 */

#ifndef DESCARTES_CORE_PATH_PLANNER_BASE_H_
#define DESCARTES_CORE_PATH_PLANNER_BASE_H_

#include <descartes_core/trajectory_pt.h>
#include <descartes_core/robot_model.h>
#include <vector>

namespace descartes_core
{
namespace PlannerErrors
{
enum PlannerError
{
  OK = 1,
  IK_NOT_AVAILABLE= -1,
  FX_NOT_AVAILABLE = -2,
  SELF_COLLISION_FOUND = -3 ,
  ENVIRONMENT_COLLISION_FOUND = -4  ,
  PLANNING_TIMEOUT = -5,
  EMPTY_PATH = -6,
  SPEED_LIMIT_EXCEEDED = -7,
  ACCELERATION_LIMIT_EXCEEDED = -8,
  MAX_TRAJECTORY_SIZE_EXCEEDED = -9 ,
  UNINITIALIZED = -10,
  INVALID_ID = -11,
  INCOMPLETE_PATH = -12,
  INVALID_CONFIGURATION_PARAMETER = -13,
  UKNOWN = -99
};
}
typedef PlannerErrors::PlannerError PlannerError;

typedef std::map<std::string,std::string> PlannerConfig;


DESCARTES_CLASS_FORWARD(PathPlannerBase);
class PathPlannerBase
{
public:
  virtual ~PathPlannerBase(){}

  /**
   * @brief Plans a path for the given robot model and configuration parameters.
   * @param model robot model implementation for which to plan a path
   * @param config A map containing the parameter/value pairs.
   */
  virtual bool initialize(RobotModelConstPtr model) = 0;

  /**
   * @brief Configure the planner's parameters. Should return 'true' when all the entries were properly parsed.
   * @param config A map containing the parameter/value pairs.
   */
  virtual bool setConfig( const PlannerConfig& config) = 0;

  /**
   * @brief Get the current configuration parameters used by the planner
   * @param config A map containing the current parameter/value pairs.
   */
  virtual void getConfig(PlannerConfig& config) const = 0;

  /**
   * @brief Generates a robot path from the trajectory.
   * @param traj the points used to plan the robot path
   * @error_code Integer flag which indicates the type of error encountered during planning.
   */
  virtual bool planPath(const std::vector<TrajectoryPtPtr>& traj) = 0;

  /**
   * @brief Returns the last robot path generated from the input trajectory
   * @param path Array that contains the points in the robot path
   */
  virtual bool getPath(std::vector<TrajectoryPtPtr>& path) const = 0;

  /**
   * @brief Add a point to the current path after the point with 'ref_id'.
   * @param ref_id ID of the reference point
   */
  virtual bool addAfter(const TrajectoryPt::ID& ref_id,TrajectoryPtPtr tp) = 0;

  /**
   * @brief Add a point to the current path before the point with 'ref_id'.
   * @param ref_id ID of the reference point
   */
  virtual bool addBefore(const TrajectoryPt::ID& ref_id,TrajectoryPtPtr tp) = 0;

  /**
   * @brief Removes the point with 'ref_id' from the path.
   * @param ref_id ID of the reference point
   */
  virtual bool remove(const TrajectoryPt::ID& ref_id) = 0;

  /**
   * @brief Modifies the point with 'ref_id'.
   * @param ref_id ID of the reference point
   */
  virtual bool modify(const TrajectoryPt::ID& ref_id,TrajectoryPtPtr tp) = 0;

  /**
   * @brief Returns the last error code.
   */
  virtual int getErrorCode() const = 0;

  /**
   * @brief Gets the error message corresponding to the error code.
   * @param error_code Integer code from the PlannerError enumeration.
   */
  virtual bool getErrorMessage(int error_code, std::string& msg) const = 0;

protected:
  PathPlannerBase(){}
};

}

#endif /* DESCARTES_CORE_PATH_PLANNER_BASE_H_ */
