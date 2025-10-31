#pragma once

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <rviz_common/display_context.hpp>
#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/view_manager.hpp>
#include <QPointer>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <dua_rviz_plugins/displays/attitude_indicator/attitude_indicator_widget.hpp>

namespace dua_rviz_plugins::displays::attitude_indicator
{

class AttitudeIndicatorDisplay
  : public rviz_common::MessageFilterDisplay<geometry_msgs::msg::PoseStamped>
{
  Q_OBJECT

public:
  AttitudeIndicatorDisplay();
  ~AttitudeIndicatorDisplay() override;

protected:
  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;
  void reset() override;
  void update(float wall_dt, float ros_dt) override;
  void processMessage(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) override;

private:
  void ensureOverlay();

  // Properties
  rviz_common::properties::FloatProperty * prop_scale_ = nullptr;
  rviz_common::properties::IntProperty *   prop_margin_ = nullptr;
  rviz_common::properties::EnumProperty *  prop_corner_ = nullptr;

  // Internal variables
  QPointer<AttitudeIndicatorWidget> overlay_ = nullptr;
  double roll_ = 0.0, pitch_ = 0.0, yaw_ = 0.0;
};

} // namespace dua_rviz_plugins::displays::attitude_indicator
