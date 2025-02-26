/**
 * TextPubPanel header file.
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

 #ifndef DUA_RVIZ_PLUGINS__TEXT_PUB_PANEL_HPP_
 #define DUA_RVIZ_PLUGINS__TEXT_PUB_PANEL_HPP_

 // ROS2 libraries
 #include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

// Qt libraries
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>

// Messages
#include <std_msgs/msg/string.hpp>

#define PUB_QUEUE_SIZE 10

namespace dua_rviz_plugins
{

using std_msgs::msg::String;

/**
 * @brief Custom panel for sending overlay text messages in RViz.
 */
class TextPubPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  /**
   * @brief Constructor.
   *
   * @param parent The parent widget.
   */
  TextPubPanel(QWidget * parent = nullptr);

  /**
   * @brief Initializes the panel when it is added to RViz.
   */
  void onInitialize() override;

private Q_SLOTS:
  /**
   * @brief Updates the topic name and creates a new publisher if necessary.
   */
  void updateTopic();

  /**
   * @brief Sends the message to the selected topic.
   */
  void sendMessage();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  QLabel * topic_label_;
  QLabel * status_label_;
  QLineEdit * topic_input_;
  QLineEdit * message_input_;
  QPushButton * send_button_;
  QString topic_name_ = "/messages";
};

}  // namespace dua_rviz_plugins

#endif  // DUA_RVIZ_PLUGINS__TEXT_PUB_PANEL_HPP_
