#pragma once

#include <noether_gui/widgets.h>
#include <noether_tpp/core/mesh_modifier.h>
#include <QWidget>
#include <rclcpp/client.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/node.hpp>
#include <rviz_polygon_selection_tool/srv/get_selection.hpp>

namespace Ui
{
class ROISelectionMeshModifier;
}

namespace snp_tpp
{
class ROISelectionMeshModifierWidget : public noether::MeshModifierWidget
{
  Q_OBJECT
public:
  ROISelectionMeshModifierWidget(QWidget* parent = nullptr);
  virtual ~ROISelectionMeshModifierWidget();

  noether::MeshModifier::ConstPtr create() const override;

  void configure(const YAML::Node& config) override;
  void save(YAML::Node& config) const override;

private:
  void spin();

  Ui::ROISelectionMeshModifier* ui_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<rviz_polygon_selection_tool::srv::GetSelection>::SharedPtr client_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread thread_;
};

}  // namespace snp_tpp
