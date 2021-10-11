/*
 * myworkcell_core.cpp
 *
 *  Created on: Sep 12, 2019
 *      Author: Jorge Nicho
 */

#include <rclcpp/rclcpp.hpp>
#include <myworkcell_msgs/srv/localize_part.hpp>
#include <myworkcell_msgs/srv/plan_cartesian_path.hpp>
#include <myworkcell_msgs/srv/move_to_pose.hpp>
#include <myworkcell_msgs/srv/execute_trajectory.hpp>
#include <console_bridge/console.h>

static const std::string NODE_NAME = "myworkcell_node";
static const int WAIT_SERVICE_PERIOD = 10;

class ScanNPlan: public rclcpp::Node
{
public:
  ScanNPlan(const rclcpp::NodeOptions & options):
    rclcpp::Node(NODE_NAME, options)
  {
    //***FILL CODE HERE IN CONSTRUCTOR FOR SERVICES

  }

  ~ScanNPlan()
  {

  }

  bool start(const std::string& base_frame)
  {
    //***FILL CODE HERE TO CALL THE ROS SERVICES OVER THE BRIDGE
    
  }
  
  //***FILL CODE HERE TO DECLARE SERVICES

};

int main(int argc, char** argv)
{
  //***FILL CODE HERE TO START ROS2, INSTANTIATE THE ScanNPlan CLASS, AND CALL THE start() METHOD

}





