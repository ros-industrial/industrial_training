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
#include <console_bridge/console.h>

static const std::string NODE_NAME = "myworkcell_node";

class ScanNPlan: public rclcpp::Node
{
public:
  ScanNPlan(const rclcpp::NodeOptions & options):
    rclcpp::Node(NODE_NAME, options)
  {
    vision_client_ = this->create_client<myworkcell_msgs::srv::LocalizePart>("localize_part");
    cartesian_client_ = this->create_client<myworkcell_msgs::srv::PlanCartesianPath>("plan_path");
    move_client_ = this->create_client<myworkcell_msgs::srv::MoveToPose>("move_to_pose");

  }

  ~ScanNPlan()
  {

  }

  bool start(const std::string& base_frame)
  {
    using namespace myworkcell_msgs::srv;

    // waiting for services
    std::vector<rclcpp::ClientBase* > clients = {vision_client_.get(), cartesian_client_.get() , move_client_.get()};
    if(std::all_of(clients.begin(), clients.end(),[](rclcpp::ClientBase* c){
      std::cout<<"Waiting for client "<< c->get_service_name()<<std::endl;
      return c->wait_for_service(std::chrono::seconds(2));
    }))
    {
      std::cout<<"Found all services"<<std::endl;
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "The service was not found");
      return false;
    }

    // sending localizatio request
    LocalizePart::Request::SharedPtr req = std::make_shared<LocalizePart::Request>();
    req->base_frame = base_frame;
    std::cout<<"Requesting pose in base frame: "<< base_frame << std::endl;
    //RCLCPP_INFO(this->get_logger(),"Requesting pose in base frame: %s", base_frame.c_str());

    std::shared_future<LocalizePart::Response::SharedPtr> result_future = vision_client_->async_send_request(req);
    if(rclcpp::spin_until_future_complete(this->get_node_base_interface(),result_future) != rclcpp::executor::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Could not localize part");
      return false;
    }

    // getting response
    LocalizePart::Response::SharedPtr res = result_future.get();
    std::cout<<"Part localized"<<std::endl;
    geometry_msgs::msg::Pose move_target = res->pose;

    // moving to pose
    MoveToPose::Request::SharedPtr move_req = std::make_shared<MoveToPose::Request>();
    move_req->pose.header.frame_id = base_frame;
    move_req->pose.pose = move_target;
    std::shared_future<MoveToPose_Response::SharedPtr> move_future = move_client_->async_send_request(move_req);
    if(rclcpp::spin_until_future_complete(this->get_node_base_interface(),move_future) != rclcpp::executor::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(),"Failed to move to target");
      return false;
    }

    // planning cartesian path
    PlanCartesianPath_Request::SharedPtr plan_req = std::make_shared<PlanCartesianPath::Request>();
    plan_req->pose = move_target;
    std::shared_future<PlanCartesianPath_Response::SharedPtr> plan_future = cartesian_client_->async_send_request(plan_req);
    if(rclcpp::spin_until_future_complete(this->get_node_base_interface(),plan_future) != rclcpp::executor::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(),"Could not plan for path");
      return false;
    }

    // executing
    return true;
  }

  rclcpp::Client<myworkcell_msgs::srv::LocalizePart>::SharedPtr vision_client_;
  rclcpp::Client<myworkcell_msgs::srv::PlanCartesianPath>::SharedPtr cartesian_client_;
  rclcpp::Client<myworkcell_msgs::srv::MoveToPose>::SharedPtr move_client_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc,argv);
  std::shared_ptr<ScanNPlan> app = std::make_shared<ScanNPlan>(rclcpp::NodeOptions());

  // getting parameter
  rclcpp::ParameterValue base_frame_param = app->declare_parameter("base_frame");
  if(base_frame_param.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET)
  {
    RCLCPP_ERROR(app->get_logger(),"Parameter not found");
    return -1;
  }
  std::string base_frame = base_frame_param.get<std::string>();
  std::cout<<"Got base_frame parameter " << base_frame << std::endl;

  std::thread t([&](){
    rclcpp::spin(app);
  });

  if(!app->start(base_frame))
  {
    rclcpp::shutdown();
    return -1;
  }
  t.join();
  rclcpp::shutdown();
  return 0;
}





