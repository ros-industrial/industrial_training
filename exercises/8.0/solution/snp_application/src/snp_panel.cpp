#include "snp_widget.h"

#include <QMessageBox>
#include <QVBoxLayout>
#include <rviz_common/panel.hpp>
#include <rviz_common/display_context.hpp>

class SNPPanel : public rviz_common::Panel
{
public:
  void onInitialize() override
  {
    try
    {
      auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

      auto layout = new QVBoxLayout();
      layout->addWidget(new SNPWidget(node, this));
      setLayout(layout);
    }
    catch (const std::exception& ex)
    {
      QMessageBox::warning(this, "Error", ex.what());
    }
  }
};

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(::SNPPanel, rviz_common::Panel)
