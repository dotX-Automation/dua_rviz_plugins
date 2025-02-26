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

// ROS2 libraries
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

// Qt libraries
#include <QLabel>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>

// C++ libraries
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

// Messages
#include <std_msgs/msg/string.hpp>

#define SUB_QUEUE_SIZE 10
#define TEXT_SIZE 20

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
   * @brief Initializes the panel when it is added to RViz.
   */
  void onInitialize() override;

  /**
   * @brief Saves the panel configuration, including the selected topic.
   *
   * @param config The configuration object to store the topic name.
   */
  void save(rviz_common::Config config) const override;

  /**
   * @brief Loads the panel configuration and restores the topic subscription.
   *
   * @param config The configuration object containing the stored topic name.
   */
  void load(const rviz_common::Config & config) override;

private Q_SLOTS:
  /**
   * @brief Updates the topic subscription based on user input.
   */
  void updateTopic();

private:
  /**
   * @brief Subscribes to the selected ROS 2 topic.
   */
  void subscribe();

  /**
   * @brief Unsubscribes from the current topic to release resources.
   */
  void unsubscribe();

  /**
   * @brief Callback function that updates the UI with the received boolean message.
   *
   * @param msg The received message containing the string data.
   */
  void callback(const std_msgs::msg::String::SharedPtr msg);

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Node::SharedPtr node_;
  QLabel * label_;
  QLabel * topic_info_label_;
  QLineEdit * topic_input_;
  QString topic_name_ = "";
  std::map<std::string, std::string> color_map_;
};

}  // namespace dua_rviz_plugins

#endif  // DUA_RVIZ_PLUGINS__TEXT_SUB_PANEL_HPP_