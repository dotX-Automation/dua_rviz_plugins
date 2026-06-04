#pragma once

#include <mutex>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/view_manager.hpp>
#include <QPointer>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <dua_rviz_plugins/displays/attitude_indicator/attitude_indicator_widget.hpp>

namespace dua_rviz_plugins::displays::attitude_indicator
{

class AttitudeIndicatorDisplay : public rviz_common::Display
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

private Q_SLOTS:
  void onMsgTypeChanged();
  void onTopicChanged();

private:
  void ensureOverlay();
  void resubscribe();
  void unsubscribeAll();
  void processQuaternion(double x, double y, double z, double w);

  void cbImu(sensor_msgs::msg::Imu::ConstSharedPtr msg);
  void cbOdometry(nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void cbPoseStamped(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);

  // Properties
  rviz_common::properties::EnumProperty *     prop_msg_type_ = nullptr;
  rviz_common::properties::RosTopicProperty * prop_topic_    = nullptr;
  rviz_common::properties::FloatProperty *    prop_scale_    = nullptr;
  rviz_common::properties::IntProperty *      prop_margin_   = nullptr;
  rviz_common::properties::EnumProperty *     prop_corner_   = nullptr;

  // Subscriptions — at most one is non-null at runtime
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr           sub_imu_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr         sub_odom_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;

  // Internal state
  QPointer<AttitudeIndicatorWidget> overlay_ = nullptr;
  std::mutex angles_mutex_;
  double roll_ = 0.0, pitch_ = 0.0, yaw_ = 0.0;
};

} // namespace dua_rviz_plugins::displays::attitude_indicator
