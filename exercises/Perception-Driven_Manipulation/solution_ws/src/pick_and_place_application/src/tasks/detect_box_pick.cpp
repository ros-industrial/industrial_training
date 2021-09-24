#include <pick_and_place_application/pick_and_place.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/* DETECTING BOX PICK POSE
  Goal:
    - Find the box's position in the world frame using the transform listener.
        * this transform is published by the kinect AR-tag perception node
    - Save the pose into 'box_pose'.
*/
geometry_msgs::msg::Pose pick_and_place_application::PickAndPlaceApp::detectBox()
{
  using RequestType = pick_and_place_msgs::srv::GetTargetPose::Request;
  using ResponseType = pick_and_place_msgs::srv::GetTargetPose::Response;

  //RCLCPP_ERROR_STREAM(node,"detect_box_pick is not implemented yet.  Aborting."); exit(1);

  // creating shape for recognition
  shape_msgs::msg::SolidPrimitive shape;
  shape.type = shape_msgs::msg::SolidPrimitive::BOX;
  shape.dimensions.resize(3);
  shape.dimensions[0] = cfg.BOX_SIZE.getX();
  shape.dimensions[1] = cfg.BOX_SIZE.getY();
  shape.dimensions[2] = cfg.BOX_SIZE.getZ();

  // creating request object
  RequestType::SharedPtr req = std::make_shared<RequestType>();
  req->shape = shape;
  req->world_frame_id = cfg.WORLD_FRAME_ID;
  req->ar_tag_frame_id = cfg.AR_TAG_FRAME_ID;

  /* Fill Code:
   * Goal:
   * - Call target recognition service and save results.
   * Hint:
   * - Use the service response member to access the
   * 	detected pose "srv.response.target_pose".
   * - Assign the target_pose in the response to the box_pose variable in
   * 	order to save the results.
   */
  geometry_msgs::msg::Pose box_pose;
  std::shared_future<ResponseType::SharedPtr> response_fut = target_recognition_client->async_send_request(req);
  std::future_status st = response_fut.wait_for(
      rclcpp::Duration::from_seconds(20.0).to_chrono<std::chrono::seconds>());
  if(st == std::future_status ::ready)
  {
    ResponseType::SharedPtr response = response_fut.get();
    if(response->succeeded)
    {
      box_pose = response->target_pose;
      RCLCPP_INFO_STREAM(node->get_logger(),"target recognition succeeded");
    }
    else
    {
      RCLCPP_ERROR_STREAM(node->get_logger(),"target recognition failed");
      exit(0);
    }
  }
  else
  {
    RCLCPP_ERROR_STREAM(node->get_logger(),"Service call for target recognition failed with response timed out");
    exit(0);
  }

  // updating box marker for visualization in rviz
  cfg.MARKER_MESSAGE.header.frame_id = cfg.WORLD_FRAME_ID;
  cfg.MARKER_MESSAGE.pose = box_pose;
  cfg.MARKER_MESSAGE.pose.position.z = box_pose.position.z - 0.5f*cfg.BOX_SIZE.z();

  showBox(true);

  return box_pose;
}

