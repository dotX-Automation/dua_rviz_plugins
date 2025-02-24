#ifndef DUA_RVIZ_PLUGINS__START_STOP_PANEL_HPP_
#define DUA_RVIZ_PLUGINS__START_STOP_PANEL_HPP_

#include <dua_qos_cpp/dua_qos.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/empty.hpp>

#include <QPushButton>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QLineEdit>
#include <QLabel>
#include <QString>

namespace dua_rviz_plugins
{

/**
 * @brief Custom button for sending start and stop text messages in RViz.
 */
class StartStopPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  /**
   * @brief Constructor for StartStopPanel.
   */
  StartStopPanel(QWidget * parent = nullptr);

private Q_SLOTS:
  /**
   * @brief Publishes an empty message on the first topic.
   */
  void startButtonClicked();
  /**
   * @brief Publishes an empty message on the second topic.
   */
  void stopButtonClicked();
  /**
   * @brief Updates the publisher for the start topic when the topic name is changed from the GUI.
   */
  void updateStartTopic();
  /**
   * @brief Updates the publisher for the stop topic when the topic name is changed from the GUI.
   */
  void updateStopTopic();

private:
  rclcpp::Node::SharedPtr node_; /**< The ROS node. >*/
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher_start_; /**< The publisher for the first topic. >*/
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher_stop_; /**< The publisher for the second topic. >*/
  QPushButton * button_start_; /**< Button for starting the process. >*/
  QPushButton * button_stop_; /**< Button for stopping the process. >*/
  QLineEdit * topic_start_input_; /**< Input field for the start topic name. >*/
  QLineEdit * topic_stop_input_; /**< Input field for the stop topic name. >*/
  QString topic_start_name_; /**< Current start topic name. >*/
  QString topic_stop_name_; /**< Current stop topic name. >*/
};

} // namespace dua_rviz_plugins

#endif  // DUA_RVIZ_PLUGINS__START_STOP_HPP_
