#ifndef ROSCONWINDOW_H
#define ROSCONWINDOW_H

#include <QWidget>
#include <rclcpp/node.hpp>
#include <rclcpp_action/client.hpp>
// Messages
#include <geometry_msgs/msg/pose_array.hpp>
#include <industrial_reconstruction_msgs/srv/start_reconstruction.hpp>
#include <industrial_reconstruction_msgs/srv/stop_reconstruction.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <snp_msgs/srv/generate_tool_paths.hpp>
#include <snp_msgs/srv/generate_motion_plan.hpp>
#include <snp_msgs/srv/execute_motion_plan.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace Ui
{
class SNPWidget;
}

class SNPWidget : public QWidget
{
  Q_OBJECT

public:
  explicit SNPWidget(rclcpp::Node::SharedPtr node, QWidget* parent = nullptr);

private:
  Ui::SNPWidget* ui_;
  bool past_calibration_;

  const std::string mesh_file_;
  const std::string motion_group_;
  const std::string reference_frame_;
  const std::string tcp_frame_;
  const std::string camera_frame_;
  const trajectory_msgs::msg::JointTrajectory scan_traj_;
  industrial_reconstruction_msgs::srv::StartReconstruction::Request::SharedPtr start_scan_request_;

  // joint state publisher
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr toolpath_pub_;

  // service clients
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr observe_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr run_calibration_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr get_correlation_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr install_calibration_client_;

  rclcpp::Client<industrial_reconstruction_msgs::srv::StartReconstruction>::SharedPtr start_reconstruction_client_;
  rclcpp::Client<industrial_reconstruction_msgs::srv::StopReconstruction>::SharedPtr stop_reconstruction_client_;

  rclcpp::Client<snp_msgs::srv::GenerateToolPaths>::SharedPtr tpp_client_;
  rclcpp::Client<snp_msgs::srv::GenerateMotionPlan>::SharedPtr motion_planning_client_;

  rclcpp::Client<snp_msgs::srv::ExecuteMotionPlan>::SharedPtr motion_execution_client_;

  /**
   * @brief Updates the GUI to reflect the status of the internal state machine
   * @details This method updates elements of the GUI and can only be called from the Qt thread, not in ROS callbacks.
   * To invoke this method from a ROS callback, emit the `updateStatus` signal
   */
  void onUpdateStatus(bool success, QString current_process, QString next_process, unsigned step);
  void onUpdateLog(const QString& message);

  bool scan_complete_{ false };
  snp_msgs::msg::ToolPaths::SharedPtr tool_paths_{ nullptr };
  trajectory_msgs::msg::JointTrajectory::SharedPtr motion_plan_{ nullptr };

  void update_calibration_requirement();
  void observe();
  void run_calibration();
  void get_correlation();
  void install_calibration();
  void reset_calibration();

  // Scan motion and reconstruction
  using FJTResult = rclcpp::Client<snp_msgs::srv::ExecuteMotionPlan>::SharedFuture;
  using StartScanFuture = rclcpp::Client<industrial_reconstruction_msgs::srv::StartReconstruction>::SharedFuture;
  using StopScanFuture = rclcpp::Client<industrial_reconstruction_msgs::srv::StopReconstruction>::SharedFuture;
  void scan();
  void onScanApproachDone(FJTResult result);
  void onScanStartDone(StartScanFuture result);
  void onScanDone(FJTResult result);
  void onScanStopDone(StopScanFuture result);
  void onScanDepartureDone(FJTResult result);

  void planToolPaths();
  void onPlanToolPathsDone(rclcpp::Client<snp_msgs::srv::GenerateToolPaths>::SharedFuture result);

  void planMotion();
  void onPlanMotionDone(rclcpp::Client<snp_msgs::srv::GenerateMotionPlan>::SharedFuture result);

  void execute();
  void onExecuteDone(rclcpp::Client<snp_msgs::srv::ExecuteMotionPlan>::SharedFuture result);

  void reset();

signals:
  void log(const QString& message);
  void updateStatus(bool success, QString current_process, QString next_process, unsigned step);
};

#endif  // ROSCONWINDOW_H
