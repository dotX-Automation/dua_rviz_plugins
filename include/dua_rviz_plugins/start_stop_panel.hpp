#ifndef DUA_RVIZ_PLUGINS__START_STOP_PANEL_HPP_
#define DUA_RVIZ_PLUGINS__START_STOP_PANEL_HPP_

#include <dua_qos_cpp/dua_qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/empty.hpp>

#include <QPushButton>
#include <QHBoxLayout>
#include <QVBoxLayout>

namespace dua_rviz_plugins
{

class StartStopPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  StartStopPanel(QWidget * parent = nullptr);

private Q_SLOTS:
  void startButtonClicked();
  void stopButtonClicked();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher_start_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher_stop_;
  QPushButton * button_start_;
  QPushButton * button_stop_;
};

} // namespace dua_rviz_plugins

#endif  // DUA_RVIZ_PLUGINS__START_STOP_HPP_
