#include <tf2_eigen/tf2_eigen.h>
#include <pick_and_place_application/pick_and_place.h>

/*    SET ATTACHED OBJECT
  Goal:
    - Attaches or detaches a box payload to the arm.
  Hints:
    - See how to ask moveit to provide the current state of the robot using the
        "getCurrentState()" method in the "moveit_cpp" object.
    - See how to add or remove the payload from the robot state object.
*/

namespace pick_and_place_application
{
void PickAndPlaceApp::setAttachedObject(bool attach,
                                        const geometry_msgs::msg::Pose& pose,
                                        moveit_msgs::msg::RobotState& robot_state)
{
  // get robot state
  moveit::core::RobotStatePtr current_state = moveit_cpp->getCurrentState(3.0);

  if (attach)
  {
    // constructing shape
    std::vector<shapes::ShapeConstPtr> shapes_array;
    shapes::ShapeConstPtr shape(shapes::constructShapeFromMsg(cfg.ATTACHED_OBJECT.primitives[0]));
    shapes_array.push_back(shape);

    // constructing pose
    EigenSTL::vector_Isometry3d pose_array(1);
    tf2::fromMsg(cfg.ATTACHED_OBJECT.primitive_poses[0], pose_array[0]);

    // attaching
    current_state->attachBody(
        cfg.ATTACHED_OBJECT_LINK_NAME, Eigen::Isometry3d::Identity(), shapes_array, pose_array, cfg.TOUCH_LINKS, cfg.TCP_LINK_NAME);

    // update box marker
    cfg.MARKER_MESSAGE.header.frame_id = cfg.TCP_LINK_NAME;
    cfg.MARKER_MESSAGE.pose = cfg.TCP_TO_BOX_POSE;
  }
  else
  {
    // detaching
    if (current_state->hasAttachedBody(cfg.ATTACHED_OBJECT_LINK_NAME))
      current_state->clearAttachedBody(cfg.ATTACHED_OBJECT_LINK_NAME);
  }

  // save robot state data
  moveit::core::robotStateToRobotStateMsg(*current_state, robot_state);
}

}  // namespace pick_and_place_application
