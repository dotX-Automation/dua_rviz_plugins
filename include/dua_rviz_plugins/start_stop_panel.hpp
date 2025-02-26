/**
 * StartStopPanel header file.
 *
 * dotX Automation <info@dotxautomation.com>
 *
 *  February 17, 2025
 */

/**
 * Copyright 2024 dotX Automation s.r.l.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

 #ifndef DUA_RVIZ_PLUGINS__START_STOP_PANEL_HPP_
 #define DUA_RVIZ_PLUGINS__START_STOP_PANEL_HPP_

// DUA libraries
#include <dua_qos_cpp/dua_qos.hpp>

// ROS2 libraries
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

// Qt libraries
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QString>
#include <QVBoxLayout>

// Messages
#include <std_msgs/msg/empty.hpp>

#define REPUBLISH_COUNT 10

namespace dua_rviz_plugins
{

using std_msgs::msg::Empty;

/**
 * @brief Custom button for sending start and stop text messages in RViz.
 */
class StartStopPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  /**
   * @brief Constructor
   */
  StartStopPanel(QWidget * parent = nullptr);

  /**
   * @brief Initializes the panel.
   */
  void onInitialize() override;

private Q_SLOTS:
  /**
   * @brief Initializes the publisher for the start button.
   *
   * @param start_topic_str The name of the start topic.
   */
  void init_start_pub(const std::string & start_topic_str);

  /**
   * @brief Initializes the publisher for the stop button.
   *
   * @param stop_topic_str The name of the stop topic.
   */
  void init_stop_pub(const std::string & stop_topic_str);

  /**
   * @brief Publishes an empty message to the start topic.
   */
  void start_button_clicked();

  /**
   * @brief Publishes an empty message to the stop topic.
   */
  void stop_button_clicked();

  /**
   * @brief Updates the publisher for the start topic when the topic name is changed from the GUI.
   */
  void update_start_topic();

  /**
   * @brief Updates the publisher for the stop topic when the topic name is changed from the GUI.
   */
  void update_stop_topic();

private:
  rclcpp::Node::SharedPtr node_;
  QString start_topic_;
  QString stop_topic_;
  rclcpp::Publisher<Empty>::SharedPtr start_pub_;
  rclcpp::Publisher<Empty>::SharedPtr stop_pub_;
  QLineEdit * start_topic_input_;
  QLineEdit * stop_topic_input_;
};

} // namespace dua_rviz_plugins

#endif  // DUA_RVIZ_PLUGINS__START_STOP_PANEL_HPP_
