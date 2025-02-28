/**
 * TextSubPanel header file.
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

 #ifndef DUA_RVIZ_PLUGINS__TEXT_SUB_PANEL_HPP_
 #define DUA_RVIZ_PLUGINS__TEXT_SUB_PANEL_HPP_

// DUA libraries
#include <dua_qos_cpp/dua_qos.hpp>

// ROS2 libraries
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

// Qt libraries
#include <QLabel>
#include <QLineEdit>
#include <QString>
#include <QVBoxLayout>

// C++ libraries
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

// Messages
#include <std_msgs/msg/string.hpp>

#define FONT_SIZE 20

namespace dua_rviz_plugins
{

using std_msgs::msg::String;

/**
 * @brief Custom panel for displaying overlay text messages in RViz.
 */
class TextSubPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  /**
   * @brief Constructor.
   *
   * @param parent The parent widget.
   */
  TextSubPanel(QWidget * parent = nullptr);

  /**
   * @brief Initializes the panel.
   */
  void onInitialize() override;

private Q_SLOTS:

  /**
   * @brief Initializes the subscriber.
   *
   * @param topic_str The name of the topic.
   */
  void init_sub(const std::string & topic_str);

  /**
   * @brief Callback function that updates the UI with the received boolean message.
   *
   * @param msg The received message containing the string data.
   */
  void show_message_received(const std_msgs::msg::String::SharedPtr msg);

  /**
   * @brief Updates the topic name based on user input.
   */
  void update_topic();

private:
  rclcpp::Node::SharedPtr node_;
  QString topic_;
  rclcpp::Subscription<String>::SharedPtr sub_;
  QLineEdit * topic_input_;
  QLabel * message_label_;
  std::map<std::string, std::string> color_map_;
};

}  // namespace dua_rviz_plugins

#endif  // DUA_RVIZ_PLUGINS__TEXT_SUB_PANEL_HPP_
