#include <pick_and_place_application/pick_and_place.h>

/*    RESET WORLD
  Goal:
  Hints:
*/
void pick_and_place_application::PickAndPlaceApp::resetWorld(bool do_perception)
{
  // detach box if one is attached
  moveit_msgs::msg::RobotState robot_state_msg;
  setAttachedObject(false, geometry_msgs::msg::Pose(), robot_state_msg);

  // get new sensor snapshot
  if (do_perception)
  {
    detectBox();
  }

  showBox(false);
}
