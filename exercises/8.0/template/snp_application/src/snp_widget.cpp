#include "snp_widget.h"
#include "ui_snp_widget.h"

#include <QMessageBox>
#include <QScrollBar>
#include <QTextStream>
#include <rclcpp_action/create_client.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include "serialize.h"
#include "trajectory_msgs_yaml.h"

static const std::string TOOL_PATH_TOPIC = "toolpath";

static const std::string CALIBRATION_OBSERVE_SERVICE = "observe";
static const std::string CALIBRATION_RUN_SERVICE = "run";
static const std::string CALIBRATION_CORRELATION_SERVICE = "correlation";
static const std::string CALIBRATION_INSTALL_SERVICE = "install";
static const std::string START_RECONSTRUCTION_SERVICE = "start_reconstruction";
static const std::string STOP_RECONSTRUCTION_SERVICE = "stop_reconstruction";
static const std::string GENERATE_TOOL_PATHS_SERVICE = "generate_tool_paths";
static const std::string MOTION_PLAN_SERVICE = "create_motion_plan";
static const std::string MOTION_EXECUTION_SERVICE = "execute_motion_plan";

static const QString CALIBRATION_ST = "calibrate";
static const QString SCAN_APPROACH_ST = "execute scan approach";
static const QString START_RECONSTRUCTION_ST = "start reconstruction";
static const QString SCAN_EXECUTION_ST = "execute scan";
static const QString STOP_RECONSTRUCTION_ST = "stop reconstruction";
static const QString SCAN_DEPARTURE_ST = "execute scan departure";
static const QString TPP_ST = "plan tool paths";
static const QString MOTION_PLANNING_ST = "perform motion planning";
static const QString MOTION_EXECUTION_ST = "execute process motion";

static const std::map<QString, unsigned> STATES = {
  { CALIBRATION_ST, 0 },
  { SCAN_APPROACH_ST, 1 },
  { START_RECONSTRUCTION_ST, 2 },
  { SCAN_EXECUTION_ST, 3 },
  { STOP_RECONSTRUCTION_ST, 4 },
  { SCAN_DEPARTURE_ST, 5 },
  { TPP_ST, 6 },
  { MOTION_PLANNING_ST, 7 },
  { MOTION_EXECUTION_ST, 8 },
};

static const std::string MOTION_GROUP_PARAM = "motion_group";
static const std::string REF_FRAME_PARAM = "reference_frame";
static const std::string TCP_FRAME_PARAM = "tcp_frame";
static const std::string CAMERA_FRAME_PARAM = "camera_frame";
static const std::string MESH_FILE_PARAM = "mesh_file";
static const std::string SCAN_TRAJ_FILE_PARAM = "scan_trajectory_file";

namespace  // anonymous restricts visibility to this file
{
template <typename T>
T declareAndGet(rclcpp::Node& node, const std::string& key)
{
  T val;
  node.declare_parameter<T>(key);
  if (!node.get_parameter(key, val))
  {
    throw std::runtime_error("Failed to get '" + key + "' parameter");
  }
  return val;
}

template <typename T>
T declareAndGet(rclcpp::Node& node, const std::string& key, const T& default_value)
{
  T val;
  node.declare_parameter<T>(key);
  node.get_parameter_or(key, val, default_value);
  return val;
}

}  // namespace

SNPWidget::SNPWidget(rclcpp::Node::SharedPtr node, QWidget* parent)
  : QWidget(parent)
  , ui_(new Ui::SNPWidget)
  , past_calibration_(false)
  , mesh_file_(declareAndGet<std::string>(*node, MESH_FILE_PARAM))
  , motion_group_(declareAndGet<std::string>(*node, MOTION_GROUP_PARAM))
  , reference_frame_(declareAndGet<std::string>(*node, REF_FRAME_PARAM))
  , tcp_frame_(declareAndGet<std::string>(*node, TCP_FRAME_PARAM))
  , camera_frame_(declareAndGet<std::string>(*node, CAMERA_FRAME_PARAM))
  , scan_traj_(message_serialization::deserialize<trajectory_msgs::msg::JointTrajectory>(
        declareAndGet<std::string>(*node, SCAN_TRAJ_FILE_PARAM)))
  , start_scan_request_(std::make_shared<industrial_reconstruction_msgs::srv::StartReconstruction::Request>())
{
  ui_->setupUi(this);

  connect(ui_->calibration_group_box, &QGroupBox::clicked, this, &SNPWidget::update_calibration_requirement);
  connect(ui_->observe_button, &QPushButton::clicked, this, &SNPWidget::observe);
  connect(ui_->run_calibration_button, &QPushButton::clicked, this, &SNPWidget::run_calibration);
  connect(ui_->get_correlation_button, &QPushButton::clicked, this, &SNPWidget::get_correlation);
  connect(ui_->install_calibration_button, &QPushButton::clicked, this, &SNPWidget::install_calibration);
  connect(ui_->reset_calibration_button, &QPushButton::clicked, this, &SNPWidget::reset_calibration);
  connect(ui_->scan_button, &QPushButton::clicked, this, &SNPWidget::scan);
  connect(ui_->tpp_button, &QPushButton::clicked, this, &SNPWidget::planToolPaths);
  connect(ui_->motion_plan_button, &QPushButton::clicked, this, &SNPWidget::planMotion);
  connect(ui_->motion_execution_button, &QPushButton::clicked, this, &SNPWidget::execute);
  connect(ui_->reset_button, &QPushButton::clicked, this, &SNPWidget::reset);

  // Move the text edit scroll bar to the maximum limit whenever it is resized
  connect(ui_->text_edit_log->verticalScrollBar(), &QScrollBar::rangeChanged, [this]() {
    ui_->text_edit_log->verticalScrollBar()->setSliderPosition(ui_->text_edit_log->verticalScrollBar()->maximum());
  });

  connect(this, &SNPWidget::updateStatus, this, &SNPWidget::onUpdateStatus);
  connect(this, &SNPWidget::log, this, &SNPWidget::onUpdateLog);

  toolpath_pub_ = node->create_publisher<geometry_msgs::msg::PoseArray>(TOOL_PATH_TOPIC, 10);

  // TODO register all service/action clients
  observe_client_ = node->create_client<std_srvs::srv::Trigger>(CALIBRATION_OBSERVE_SERVICE);
  run_calibration_client_ = node->create_client<std_srvs::srv::Trigger>(CALIBRATION_RUN_SERVICE);
  get_correlation_client_ = node->create_client<std_srvs::srv::Trigger>(CALIBRATION_CORRELATION_SERVICE);
  install_calibration_client_ = node->create_client<std_srvs::srv::Trigger>(CALIBRATION_INSTALL_SERVICE);

  start_reconstruction_client_ =
      node->create_client<industrial_reconstruction_msgs::srv::StartReconstruction>(START_RECONSTRUCTION_SERVICE);
  stop_reconstruction_client_ =
      node->create_client<industrial_reconstruction_msgs::srv::StopReconstruction>(STOP_RECONSTRUCTION_SERVICE);
  tpp_client_ = node->create_client<snp_msgs::srv::GenerateToolPaths>(GENERATE_TOOL_PATHS_SERVICE);
  motion_planning_client_ = node->create_client<snp_msgs::srv::GenerateMotionPlan>(MOTION_PLAN_SERVICE);
  motion_execution_client_ = node->create_client<snp_msgs::srv::ExecuteMotionPlan>(MOTION_EXECUTION_SERVICE);

  // Populate scan request
  start_scan_request_->tracking_frame = camera_frame_;
  start_scan_request_->relative_frame = reference_frame_;
  start_scan_request_->translation_distance = 0;
  start_scan_request_->rotational_distance = 0;
  start_scan_request_->live = true;
  start_scan_request_->rgbd_params.convert_rgb_to_intensity = false;
  // Configurable parameters
  start_scan_request_->tsdf_params.voxel_length = declareAndGet<float>(*node, "tsdf.voxel_length", 0.01f);
  start_scan_request_->tsdf_params.sdf_trunc = declareAndGet<float>(*node, "tsdf.sdf_trunc", 0.03f);
  start_scan_request_->tsdf_params.min_box_values.x = declareAndGet<double>(*node, "tsdf.min.x", 0.0);
  start_scan_request_->tsdf_params.min_box_values.y = declareAndGet<double>(*node, "tsdf.min.y", 0.0);
  start_scan_request_->tsdf_params.min_box_values.z = declareAndGet<double>(*node, "tsdf.min.z", 0.0);
  start_scan_request_->tsdf_params.max_box_values.x = declareAndGet<double>(*node, "tsdf.max.x", 0.0);
  start_scan_request_->tsdf_params.max_box_values.y = declareAndGet<double>(*node, "tsdf.max.y", 0.0);
  start_scan_request_->tsdf_params.max_box_values.z = declareAndGet<double>(*node, "tsdf.max.z", 0.0);
  start_scan_request_->rgbd_params.depth_scale = declareAndGet<float>(*node, "rgbd.depth_scale", 1000.0);
  start_scan_request_->rgbd_params.depth_trunc = declareAndGet<float>(*node, "rgbd.depth_trunc", 1.1f);
}

void SNPWidget::onUpdateStatus(bool success, QString current_process, QString next_process, unsigned step)
{
  QString status;
  QTextStream status_stream(&status);
  if (success)
  {
    status_stream << current_process << " completed!";
    if (next_process != "")
    {
      status_stream << "\nWaiting to " << next_process << "...";
    }

    const double progress = (static_cast<double>(step) / static_cast<double>(STATES.size())) * 100.0;
    ui_->progress_bar->setValue(static_cast<int>(progress));
  }
  else
  {
    status_stream << current_process << " failed\nWaiting to attempt again...";
  }

  onUpdateLog(status);
}

void SNPWidget::onUpdateLog(const QString& message)
{
  ui_->text_edit_log->append(message);
}

void SNPWidget::update_calibration_requirement()
{
  if (!ui_->calibration_group_box->isChecked() && !past_calibration_)
  {
    emit updateStatus(true, CALIBRATION_ST, SCAN_APPROACH_ST, STATES.at(SCAN_APPROACH_ST));
  }
  else
  {
    reset();
  }
}

void SNPWidget::observe()
{
  if (!observe_client_->service_is_ready())
  {
    emit log("Observation service is not available");
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = observe_client_->async_send_request(request);
  future.wait();

  std_srvs::srv::Trigger::Response::SharedPtr response = future.get();
  if (response->success)
  {
    ui_->run_calibration_button->setEnabled(true);
    emit log("Gathered observation.");
  }
  else
  {
    emit log("Failed to get observation.");
  }
}

void SNPWidget::run_calibration()
{
  if (!run_calibration_client_->service_is_ready())
  {
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = run_calibration_client_->async_send_request(request);
  future.wait();
  std_srvs::srv::Trigger::Response::SharedPtr response = future.get();

  if (response->success)
  {
    ui_->install_calibration_button->setEnabled(true);
    emit log("Calibration run.");
  }
  else
  {
    emit log("Calibration attempt failed.");
  }
}

void SNPWidget::get_correlation()
{
  if (!get_correlation_client_->service_is_ready())
  {
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = get_correlation_client_->async_send_request(request);
  std_srvs::srv::Trigger::Response::SharedPtr response = future.get();

  if (response->success)
  {
    emit log("Correlation written to file.");
  }
  else
  {
    emit log("Failed to write correlation to file.");
  }
}

void SNPWidget::install_calibration()
{
  if (!install_calibration_client_->service_is_ready())
  {
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = install_calibration_client_->async_send_request(request);
  future.wait();
  std_srvs::srv::Trigger::Response::SharedPtr response = future.get();

  past_calibration_ = response->success;
  emit updateStatus(response->success, CALIBRATION_ST, SCAN_APPROACH_ST, STATES.at(SCAN_APPROACH_ST));
}

void SNPWidget::reset_calibration()
{
  past_calibration_ = false;

  // Update the UI
  ui_->run_calibration_button->setEnabled(false);
  ui_->get_correlation_button->setEnabled(false);
  ui_->install_calibration_button->setEnabled(false);
  ui_->reset_calibration_button->setEnabled(false);
}

void SNPWidget::scan()
{
  if (!motion_execution_client_->service_is_ready())
  {
    emit log("Motion execution service is not available");
    emit updateStatus(false, SCAN_APPROACH_ST, SCAN_APPROACH_ST, STATES.at(SCAN_APPROACH_ST));
    return;
  }

  emit log("Sending scan approach motion goal");

  auto request = std::make_shared<snp_msgs::srv::ExecuteMotionPlan::Request>();
  request->use_tool = false;
  request->motion_plan.header = scan_traj_.header;
  request->motion_plan.joint_names = scan_traj_.joint_names;
  request->motion_plan.points.push_back(scan_traj_.points.at(0));
  request->motion_plan.points.push_back(scan_traj_.points.at(1));

  auto cb = std::bind(&SNPWidget::onScanApproachDone, this, std::placeholders::_1);
  motion_execution_client_->async_send_request(request, cb);

  scan_complete_ = false;

  // Reset any tool paths or motion plans based on a previous scan
  tool_paths_.reset();
  motion_plan_.reset();
}

void SNPWidget::onScanApproachDone(FJTResult result)
{
  snp_msgs::srv::ExecuteMotionPlan::Response::SharedPtr response = result.get();
  if (response->success)
  {
    emit updateStatus(true, SCAN_APPROACH_ST, START_RECONSTRUCTION_ST, STATES.at(START_RECONSTRUCTION_ST));
    emit log("Successfully executed scan approach motion");
  }
  else
  {
    QString message;
    QTextStream ss(&message);
    ss << "Failed to execute scan approach motion: '" << QString::fromStdString(response->message) << "'";
    emit log(message);
    emit updateStatus(false, SCAN_APPROACH_ST, SCAN_APPROACH_ST, STATES.at(SCAN_APPROACH_ST));
    return;
  }

  auto cb = std::bind(&SNPWidget::onScanStartDone, this, std::placeholders::_1);
  start_reconstruction_client_->async_send_request(start_scan_request_, cb);
}

void SNPWidget::onScanStartDone(StartScanFuture result)
{
  if (!result.get()->success)
  {
    emit log("Failed to start surface reconstruction");
    emit updateStatus(false, START_RECONSTRUCTION_ST, SCAN_APPROACH_ST, STATES.at(SCAN_APPROACH_ST));
    return;
  }

  if (!motion_execution_client_->service_is_ready())
  {
    emit log("Motion execution service is not available");
    emit updateStatus(false, START_RECONSTRUCTION_ST, SCAN_APPROACH_ST, STATES.at(SCAN_APPROACH_ST));
    return;
  }

  emit updateStatus(true, START_RECONSTRUCTION_ST, SCAN_EXECUTION_ST, STATES.at(SCAN_EXECUTION_ST));
  emit log("Sending scan trajectory goal");

  auto request = std::make_shared<snp_msgs::srv::ExecuteMotionPlan::Request>();
  request->use_tool = false;
  request->motion_plan.header = scan_traj_.header;
  request->motion_plan.joint_names = scan_traj_.joint_names;

  const auto start_pt = scan_traj_.points.begin() + 1;
  std::transform(start_pt, scan_traj_.points.end(), std::back_inserter(request->motion_plan.points),
                 [&start_pt](trajectory_msgs::msg::JointTrajectoryPoint pt) {
                   pt.time_from_start.sec -= start_pt->time_from_start.sec;
                   pt.time_from_start.nanosec -= start_pt->time_from_start.nanosec;
                   return pt;
                 });

  auto cb = std::bind(&SNPWidget::onScanDone, this, std::placeholders::_1);
  motion_execution_client_->async_send_request(request, cb);
}

void SNPWidget::onScanDone(FJTResult result)
{
  // call reconstruction stop (regardless of trajectory success)
  auto stop_request = std::make_shared<industrial_reconstruction_msgs::srv::StopReconstruction::Request>();
  stop_request->archive_directory = "";
  stop_request->mesh_filepath = mesh_file_;
  stop_request->min_num_faces = 1000;
  industrial_reconstruction_msgs::msg::NormalFilterParams norm_filt;
  norm_filt.normal_direction.x = 0;
  norm_filt.normal_direction.y = 0;
  norm_filt.normal_direction.z = 1;
  norm_filt.angle = 85;
  stop_request->normal_filters.push_back(norm_filt);

  // Define a callback to enter when the stop reconstruction service is finished
  std::function<void(StopScanFuture)> cb;

  snp_msgs::srv::ExecuteMotionPlan::Response::SharedPtr response = result.get();
  if (response->success)
  {
    emit updateStatus(true, SCAN_EXECUTION_ST, STOP_RECONSTRUCTION_ST, STATES.at(STOP_RECONSTRUCTION_ST));
    emit log("Successfully executed scan motion");
    cb = std::bind(&SNPWidget::onScanStopDone, this, std::placeholders::_1);
  }
  else
  {
    QString message;
    QTextStream ss(&message);
    ss << "Failed to execute scan motion: '" << QString::fromStdString(response->message) << "'";
    emit log(message);
    emit updateStatus(false, SCAN_EXECUTION_ST, SCAN_APPROACH_ST, STATES.at(SCAN_APPROACH_ST));
    cb = [](StopScanFuture) {};
  }

  stop_reconstruction_client_->async_send_request(stop_request, cb);
}

void SNPWidget::onScanStopDone(StopScanFuture stop_result)
{
  if (!stop_result.get()->success)
  {
    emit log("Failed to stop surface reconstruction");
    emit updateStatus(false, STOP_RECONSTRUCTION_ST, SCAN_APPROACH_ST, STATES.at(SCAN_APPROACH_ST));
  }

  if (!motion_execution_client_->service_is_ready())
  {
    emit log("Motion execution service is not available");
    emit updateStatus(false, START_RECONSTRUCTION_ST, SCAN_APPROACH_ST, STATES.at(SCAN_APPROACH_ST));
    return;
  }

  emit updateStatus(true, STOP_RECONSTRUCTION_ST, SCAN_DEPARTURE_ST, STATES.at(SCAN_DEPARTURE_ST));
  emit log("Sending scan departure motion goal");

  auto request = std::make_shared<snp_msgs::srv::ExecuteMotionPlan::Request>();
  request->use_tool = false;
  request->motion_plan.header = scan_traj_.header;
  request->motion_plan.joint_names = scan_traj_.joint_names;

  trajectory_msgs::msg::JointTrajectoryPoint start_pt = scan_traj_.points.at(scan_traj_.points.size() - 1);
  const trajectory_msgs::msg::JointTrajectoryPoint& prev_pt = scan_traj_.points.at(scan_traj_.points.size() - 2);
  start_pt.time_from_start.sec -= prev_pt.time_from_start.sec;
  start_pt.time_from_start.nanosec -= prev_pt.time_from_start.nanosec;
  request->motion_plan.points.push_back(start_pt);

  trajectory_msgs::msg::JointTrajectoryPoint end_pt = scan_traj_.points.at(0);
  end_pt.time_from_start.sec += start_pt.time_from_start.sec;
  end_pt.time_from_start.nanosec += start_pt.time_from_start.nanosec;
  request->motion_plan.points.push_back(end_pt);

  auto cb = std::bind(&SNPWidget::onScanDepartureDone, this, std::placeholders::_1);
  motion_execution_client_->async_send_request(request, cb);
}

void SNPWidget::onScanDepartureDone(FJTResult result)
{
  snp_msgs::srv::ExecuteMotionPlan::Response::SharedPtr response = result.get();
  if (response->success)
  {
    emit log("Successfully completed scan and surface reconstruction");
    emit updateStatus(true, SCAN_DEPARTURE_ST, TPP_ST, STATES.at(TPP_ST));
    scan_complete_ = true;
  }
  else
  {
    QString message;
    QTextStream ss(&message);
    ss << "Failed to execute scan motion departure: '" << QString::fromStdString(response->message) << "'";
    emit log(message);
    emit updateStatus(false, SCAN_DEPARTURE_ST, SCAN_APPROACH_ST, STATES.at(SCAN_APPROACH_ST));
    return;
  }
}

void SNPWidget::planToolPaths()
{
  try
  {
    if (!scan_complete_)
      throw std::runtime_error("Scan has not been completed");

    if (!tpp_client_->service_is_ready())
      throw std::runtime_error("Tool path planning server is not available");

    tool_paths_.reset();

    // Reset any motion plan computed on a previous tool path
    motion_plan_.reset();

    // Fill out the service call
    auto request = std::make_shared<snp_msgs::srv::GenerateToolPaths::Request>();
    request->mesh_filename = mesh_file_;
    request->mesh_frame = reference_frame_;

    // Call the service
    auto future = tpp_client_->async_send_request(
        request, std::bind(&SNPWidget::onPlanToolPathsDone, this, std::placeholders::_1));
  }
  catch (const std::exception& ex)
  {
    emit log(QString(ex.what()));
    emit updateStatus(false, TPP_ST, TPP_ST, STATES.at(TPP_ST));
  }
}

void SNPWidget::onPlanToolPathsDone(rclcpp::Client<snp_msgs::srv::GenerateToolPaths>::SharedFuture result)
{
  snp_msgs::srv::GenerateToolPaths::Response::SharedPtr response = result.get();
  if (!response->success)
  {
    emit log(QString::fromStdString(response->message));
    emit updateStatus(true, TPP_ST, MOTION_PLANNING_ST, STATES.at(MOTION_PLANNING_ST));
    return;
  }

  tool_paths_ = std::make_shared<snp_msgs::msg::ToolPaths>(response->tool_paths);

  // Publish a message to display the tool path
  {
    geometry_msgs::msg::PoseArray flat_toolpath_msg;
    flat_toolpath_msg.header.frame_id = reference_frame_;
    for (auto& toolpath : tool_paths_->paths)
    {
      for (auto& segment : toolpath.segments)
      {
        // Update the reference frame
        segment.header.frame_id = reference_frame_;

        // Insert the waypoints into the flattened structure
        flat_toolpath_msg.poses.insert(flat_toolpath_msg.poses.end(), segment.poses.begin(), segment.poses.end());
      }
    }

    toolpath_pub_->publish(flat_toolpath_msg);
  }

  emit updateStatus(true, TPP_ST, MOTION_PLANNING_ST, STATES.at(MOTION_PLANNING_ST));
}

void SNPWidget::planMotion()
{
  emit log("Attempting motion planning");
  try
  {
    if (!tool_paths_ || tool_paths_->paths.empty())
      throw std::runtime_error("No tool paths have been defined");

    if (!motion_planning_client_->service_is_ready())
      throw std::runtime_error("Motion planning server is not available");

    // Reset the internal motion plan container
    motion_plan_.reset();

    // TODO: Fill a motion planning service request
    auto request = std::make_shared<snp_msgs::srv::GenerateMotionPlan::Request>();
    request->motion_group = motion_group_;
    request->tcp_frame = tcp_frame_;
    request->tool_paths = *tool_paths_;
    request->mesh_filename = mesh_file_;
    request->mesh_frame = reference_frame_;

    // Call the service
    motion_planning_client_->async_send_request(request,
                                                std::bind(&SNPWidget::onPlanMotionDone, this, std::placeholders::_1));

    QApplication::setOverrideCursor(Qt::BusyCursor);
  }
  catch (const std::exception& ex)
  {
    emit log(ex.what());
    emit updateStatus(false, MOTION_PLANNING_ST, MOTION_PLANNING_ST, STATES.at(MOTION_PLANNING_ST));
  }
}

void SNPWidget::onPlanMotionDone(rclcpp::Client<snp_msgs::srv::GenerateMotionPlan>::SharedFuture future)
{
  QApplication::restoreOverrideCursor();
  snp_msgs::srv::GenerateMotionPlan::Response::SharedPtr response = future.get();
  if (!response->success)
  {
    emit log(QString::fromStdString(response->message));
  }
  else
  {
    // Save the motion plan internally
    motion_plan_ = std::make_shared<trajectory_msgs::msg::JointTrajectory>(response->motion_plan);
  }

  emit updateStatus(response->success, MOTION_PLANNING_ST, MOTION_EXECUTION_ST, STATES.at(MOTION_EXECUTION_ST));
}

void SNPWidget::execute()
{
  if (!motion_plan_)
  {
    emit log("Motion plan is empty, can not execute");
    emit updateStatus(false, MOTION_EXECUTION_ST, MOTION_EXECUTION_ST, STATES.at(MOTION_EXECUTION_ST));
    return;
  }

  if (!motion_execution_client_->service_is_ready())
  {
    emit log("Motion execution service is not available");
    emit updateStatus(false, MOTION_EXECUTION_ST, MOTION_EXECUTION_ST, STATES.at(MOTION_EXECUTION_ST));
    return;
  }

  // do execution things
  auto request = std::make_shared<snp_msgs::srv::ExecuteMotionPlan::Request>();
  request->motion_plan = *motion_plan_;
  request->use_tool = true;

  motion_execution_client_->async_send_request(request,
                                               std::bind(&SNPWidget::onExecuteDone, this, std::placeholders::_1));
}

void SNPWidget::onExecuteDone(rclcpp::Client<snp_msgs::srv::ExecuteMotionPlan>::SharedFuture result)
{
  snp_msgs::srv::ExecuteMotionPlan::Response::SharedPtr response = result.get();
  if (!response->success)
  {
    QString message;
    QTextStream ss(&message);
    ss << "Motion execution error: '" << QString::fromStdString(response->message) << "'";
    emit log(message);
    emit updateStatus(response->success, MOTION_EXECUTION_ST, MOTION_EXECUTION_ST, STATES.at(MOTION_EXECUTION_ST));
    return;
  }
  else
  {
    emit updateStatus(response->success, MOTION_EXECUTION_ST, "", static_cast<unsigned>(STATES.size()));
  }
}

void SNPWidget::reset()
{
  ui_->text_edit_log->setText("Waiting to calibrate...");

  // reset button states
  ui_->run_calibration_button->setEnabled(false);
  ui_->get_correlation_button->setEnabled(false);
  ui_->install_calibration_button->setEnabled(false);
  ui_->scan_button->setEnabled(true);
  ui_->tpp_button->setEnabled(true);
  ui_->motion_plan_button->setEnabled(true);
  ui_->motion_execution_button->setEnabled(true);

  ui_->progress_bar->setValue(0);

  // Clear the internal variables
  scan_complete_ = false;
  tool_paths_.reset();
  motion_plan_.reset();
}
