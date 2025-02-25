/**
 * Reference of RViz2 TextPubPanel for DUA modules.
 *
 * Alessandro Tenaglia <a.tenaglia@dotxautomation.com>
 * Alexandru Cretu <a.cretu@dotxautomation.com>
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

#include <rviz_common/panel.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>

#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

namespace dua_rviz_plugins
{

/**
 * @brief Custom panel for sending overlay text messages in RViz.
 */
class TextPubPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  /**
   * @brief Constructor for TextPubPanel.
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
  rclcpp::Node::SharedPtr node_; /**< The ROS node. >*/
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_; /**< The publisher. >*/

  QLabel * topic_label_; /**< Label for the topic input. >*/
  QLabel * status_label_; /**< Label for the publication status. >*/
  QLineEdit * topic_input_; /**< Input for the topic name. >*/
  QLineEdit * message_input_; /**< Input for the message. >*/
  QPushButton * send_button_; /**< Button to send the message. >*/

  QString topic_name_ = "/messages"; /**< The selected topic name. >*/
};

}  // namespace dua_rviz_plugins

#endif  // DUA_RVIZ_PLUGINS__TEXT_PUB_PANEL_HPP_
