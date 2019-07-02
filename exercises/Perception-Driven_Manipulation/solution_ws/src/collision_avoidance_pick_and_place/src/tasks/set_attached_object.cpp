#include <collision_avoidance_pick_and_place/pick_and_place.h>

/*    SET ATTACHED OBJECT
  Goal:
    - Attaches or detaches a box payload to the arm.
  Hints:
    - See how to ask moveit to provide the current state of the robot using the
        "getCurrentState()" method in the "move_group_ptr" member.
    - See how to add or remove the payload from the robot state object.
*/

namespace collision_avoidance_pick_and_place
{
void PickAndPlace::set_attached_object(bool attach, const geometry_msgs::Pose &pose,moveit_msgs::RobotState &robot_state)
{
  // get robot state
  robot_state::RobotStatePtr current_state= move_group_ptr->getCurrentState();

  if(attach)
  {

    // constructing shape
    std::vector<shapes::ShapeConstPtr> shapes_array;
    shapes::ShapeConstPtr shape( shapes::constructShapeFromMsg( cfg.ATTACHED_OBJECT.primitives[0]));
    shapes_array.push_back(shape);

    // constructing pose
    tf::Transform attached_tf;
    tf::poseMsgToTF(cfg.ATTACHED_OBJECT.primitive_poses[0],attached_tf);
    EigenSTL::vector_Isometry3d pose_array(1);
    tf::transformTFToEigen(attached_tf,pose_array[0]);

    // attaching
    current_state->attachBody(cfg.ATTACHED_OBJECT_LINK_NAME,shapes_array,pose_array,cfg.TOUCH_LINKS,cfg.TCP_LINK_NAME);

    // update box marker
    cfg.MARKER_MESSAGE.header.frame_id = cfg.TCP_LINK_NAME;
    cfg.MARKER_MESSAGE.pose = cfg.TCP_TO_BOX_POSE;
  }
  else
  {

    // detaching
    if(current_state->hasAttachedBody(cfg.ATTACHED_OBJECT_LINK_NAME))
      current_state->clearAttachedBody(cfg.ATTACHED_OBJECT_LINK_NAME);
  }

  // save robot state data
  robot_state::robotStateToRobotStateMsg(*current_state,robot_state);
}

}


