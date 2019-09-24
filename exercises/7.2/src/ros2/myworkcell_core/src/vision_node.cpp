/*
 * vision_node.cpp
 *
 *  Created on: Sep 13, 2019
 *      Author: ros-industrial
 */

#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <myworkcell_msgs/srv/localize_part.hpp>
#include <eigen3/Eigen/Geometry>
#include <thread>

static const std::string NODE_NAME = "vision_node";
class Localizer: public rclcpp::Node
{
public:
  Localizer(const rclcpp::NodeOptions& opt):
    rclcpp::Node(NODE_NAME, opt),
    tf_listener_(tf_buffer_)
  {
    auto server_cb = std::bind(&Localizer::localizePart,
                       this,
                       std::placeholders::_1,
                       std::placeholders::_2);
    server_ = this->create_service<myworkcell_msgs::srv::LocalizePart>("localize_part",server_cb);

  }

  bool init()
  {
    using namespace Eigen;

    // load parameters
    std::string frame_id;
    std::vector<double> pose_vals = {-0.6, 0.2, 0.5, 0.0, 0.0, 0.0};
    const std::vector<rclcpp::ParameterValue> parameters = {this->declare_parameter("frame_id"),
                                                            this->declare_parameter("pose_vals")};
    // checking parameters
    if(!std::all_of(parameters.begin(), parameters.end(),[](const rclcpp::ParameterValue& p){
      return p.get_type() !=  rclcpp::ParameterType::PARAMETER_NOT_SET;
    }))
    {
      RCLCPP_INFO(this->get_logger(), "One or more parameters not set");
      return false;
    }

    // getting parameters now
    frame_id = parameters[0].get<std::string>();
    pose_vals = parameters[1].get<std::vector<double>>();

    Eigen::Isometry3d pose = Translation3d(pose_vals[0],pose_vals[1],pose_vals[2]) * AngleAxisd(pose_vals[3], Vector3d::UnitX()) *
        AngleAxisd(pose_vals[4], Vector3d::UnitY()) * AngleAxisd(pose_vals[5], Vector3d::UnitZ());

    // saving to member
    pose_.header.frame_id = frame_id;
    pose_.pose = tf2::toMsg(pose);

    return true;
  }

  void localizePart(const std::shared_ptr<myworkcell_msgs::srv::LocalizePart::Request> req,
                    const std::shared_ptr<myworkcell_msgs::srv::LocalizePart::Response> res)
  {

    tf2::Transform cam_to_target;
    tf2::fromMsg(pose_.pose,cam_to_target);

    geometry_msgs::msg::TransformStamped  req_to_cam_st = tf_buffer_.lookupTransform(
        req->base_frame, pose_.header.frame_id,tf2::TimePointZero);

    tf2::Transform req_to_target, req_to_cam;
    tf2::fromMsg(req_to_cam_st.transform,req_to_cam);
    req_to_target = req_to_cam * cam_to_target;

    // saving to response
    tf2::toMsg(req_to_target,res->pose);
  }
protected:
  rclcpp::Service<myworkcell_msgs::srv::LocalizePart>::SharedPtr server_;
  geometry_msgs::msg::PoseStamped pose_;
  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc,argv);
  std::shared_ptr<Localizer> localizer_node = std::make_shared<Localizer>(rclcpp::NodeOptions());
  if(!localizer_node->init())
  {
    return -1;
  }
  rclcpp::spin(localizer_node);
  rclcpp::shutdown();

  return 0;
}
