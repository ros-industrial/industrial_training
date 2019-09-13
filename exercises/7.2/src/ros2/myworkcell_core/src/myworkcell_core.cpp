/*
 * myworkcell_core.cpp
 *
 *  Created on: Sep 12, 2019
 *      Author: Jorge Nicho
 */

#include <rclcpp/rclcpp.hpp>
#include <myworkcell_msgs/srv/localize_part.hpp>
#include <thread>

static const std::string NODE_NAME = "myworkcell_node";

class ScanAndPlan: public rclcpp::Node
{
public:
  ScanAndPlan(const rclcpp::NodeOptions & options):
    rclcpp::Node(NODE_NAME, options)
  {

  }

  ~ScanAndPlan()
  {

  }

  void start(const std::string& base_frame)
  {

  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc,argv);
  std::shared_ptr<ScanAndPlan> app = std::make_shared<ScanAndPlan>(rclcpp::NodeOptions());

  // getting parameter
  std::shared_ptr<rclcpp::SyncParametersClient> parameters_client = std::make_shared<rclcpp::SyncParametersClient>(app.get());
  std::string base_frame = parameters_client->get_parameter<std::string>("base_frame", "world");

  // spinning now
  std::thread spin_thread([&](){
    rclcpp::spin(app);
  });

  app->start(base_frame);
  spin_thread.join();
  rclcpp::shutdown();
  return 0;
}





