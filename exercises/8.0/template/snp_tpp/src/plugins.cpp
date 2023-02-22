#include "roi_selection_mesh_modifier.h"

#include <noether_gui/plugin_interface.h>

namespace snp_tpp
{
struct ROISelectionMeshModifierWidgetPlugin : public noether::MeshModifierWidgetPlugin
{
  QWidget* create(QWidget* parent) const override
  {
    return new ROISelectionMeshModifierWidget(parent);
  }
};

}  // namespace snp_tpp

EXPORT_MESH_MODIFIER_WIDGET_PLUGIN(snp_tpp::ROISelectionMeshModifierWidgetPlugin, ROISelectionMeshModifier)
