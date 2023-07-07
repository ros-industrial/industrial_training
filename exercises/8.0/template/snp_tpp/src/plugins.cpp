#include <snp_tpp/roi_selection_mesh_modifier_widget.h>

#include <noether_gui/plugin_interface.h>

namespace snp_tpp
{
struct ROISelectionMeshModifierWidgetPlugin : public noether::MeshModifierWidgetPlugin
{
  QWidget* create(QWidget* parent, const YAML::Node& config = {}) const override
  {
    auto widget = new ROISelectionMeshModifierWidget(parent);

    // Attempt to configure the widget
    if (!config.IsNull())
      widget->configure(config);

    return widget;
  }
};

}  // namespace snp_tpp

EXPORT_MESH_MODIFIER_WIDGET_PLUGIN(snp_tpp::ROISelectionMeshModifierWidgetPlugin, ROISelectionMeshModifier)
