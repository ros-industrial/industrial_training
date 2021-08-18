#include <collision_avoidance_pick_and_place/pick_and_place.h>

/*    RESET WORLD
  Goal:
  Hints:
*/
void collision_avoidance_pick_and_place::PickAndPlaceApp::reset_world(bool refresh_octomap)
{

  // detach box if one is attached
  moveit_msgs::msg::RobotState robot_state;
  set_attached_object(false,geometry_msgs::msg::Pose(),robot_state);

  // get new sensor snapshot
  if(refresh_octomap)
  {
    detect_box_pick();
  }

  show_box(false);
}


