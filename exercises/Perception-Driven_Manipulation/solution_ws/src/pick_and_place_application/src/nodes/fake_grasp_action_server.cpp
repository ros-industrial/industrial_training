/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *       * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the Southwest Research Institute, nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <pick_and_place_msgs/action/execute_grasp_move.hpp>

using namespace pick_and_place_msgs;

using ExecuteGraspAction = pick_and_place_msgs::action::ExecuteGraspMove;
using ExecuteGraspActionServer = rclcpp_action::Server<ExecuteGraspAction>;
using ExecuteGraspGH = rclcpp_action::ServerGoalHandle<ExecuteGraspAction>;

static const std::string GRASP_ACTION_NAME = "do_grasp";

class GraspActionServerWrapper
{
public:
  GraspActionServerWrapper(rclcpp::Node::SharedPtr n) : node_(n)
  {
    using namespace std::placeholders;
    action_server_ =
        rclcpp_action::create_server<ExecuteGraspAction>(node_,
                                                         GRASP_ACTION_NAME,
                                                         std::bind(&GraspActionServerWrapper::goalCB, this, _1, _2),
                                                         std::bind(&GraspActionServerWrapper::cancelCB, this, _1),
                                                         std::bind(&GraspActionServerWrapper::acceptCB, this, _1));

    RCLCPP_INFO(node_->get_logger(), "Grasp execution action node started");
  }

  ~GraspActionServerWrapper() {}

private:
  rclcpp_action::GoalResponse goalCB(const rclcpp_action::GoalUUID& uuid, ExecuteGraspAction::Goal::ConstSharedPtr goal)
  {
    (void)uuid;
    (void)goal;
    RCLCPP_INFO(node_->get_logger(), "Received grasp action request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse cancelCB(const std::shared_ptr<ExecuteGraspGH> goal_handle)
  {
    RCLCPP_INFO(node_->get_logger(), "Canceling current grasp action");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void acceptCB(const std::shared_ptr<ExecuteGraspGH> goal_handle)
  {
    RCLCPP_INFO(node_->get_logger(), "Received grasping goal");
    const auto goal = goal_handle->get_goal();
    ExecuteGraspAction::Result::SharedPtr result = std::make_shared<ExecuteGraspAction::Result>();
    result->success = true;
    switch (goal->goal)
    {
      case ExecuteGraspAction::Goal::PRE_GRASP:
        RCLCPP_INFO(node_->get_logger(), "Pre-grasp command accepted");
        rclcpp::sleep_for(rclcpp::Duration::from_seconds(0.25).to_chrono<std::chrono::seconds>());
        goal_handle->succeed(result);
        break;

      case ExecuteGraspAction::Goal::GRASP:
        RCLCPP_INFO(node_->get_logger(), "Executing a gripper grasp");

        // wait
        rclcpp::sleep_for(rclcpp::Duration::from_seconds(0.25).to_chrono<std::chrono::seconds>());
        goal_handle->succeed(result);
        break;

      case ExecuteGraspAction::Goal::RELEASE:
        RCLCPP_INFO(node_->get_logger(), "Executing a gripper release");

        // wait
        rclcpp::sleep_for(rclcpp::Duration::from_seconds(0.25).to_chrono<std::chrono::seconds>());
        goal_handle->succeed(result);
        break;

      default:
        RCLCPP_INFO(node_->get_logger(), "Unidentified grasp request, ignoring");
        goal_handle->succeed(result);
        break;
    }
  }

  rclcpp::Node::SharedPtr node_;
  ExecuteGraspActionServer::SharedPtr action_server_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("fake_grasp_execution_node", "", node_options);

  // spinning node in a separate thread
  std::thread spin_thread([node]() {
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  });

  GraspActionServerWrapper action_server_wrapper(node);

  spin_thread.join();
  rclcpp::shutdown();

  return 0;
}
