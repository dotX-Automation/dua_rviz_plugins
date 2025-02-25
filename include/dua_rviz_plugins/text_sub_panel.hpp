/**
 * Reference of RViz2 TextSubPanel for DUA modules.
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

#ifndef DUA_RVIZ_PLUGINS__TEXT_SUB_PANEL_HPP_
#define DUA_RVIZ_PLUGINS__TEXT_SUB_PANEL_HPP_

#include <rviz_common/panel.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <QLabel>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

namespace dua_rviz_plugins
{

/**
 * @brief Custom panel for displaying overlay text messages in RViz.
 */
class TextSubPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  /**
   * @brief Constructor for TextSubPanel.
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

  QLabel * label_;               /**< Label to display the received message. */
  QLabel * topic_info_label_;    /**< Label to display the subscribed topic name. */
  QLineEdit * topic_input_;      /**< Input field for setting the ROS topic name. */

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_; /**< ROS 2 subscription for receiving messages. */
  rclcpp::Node::SharedPtr node_; /**< Shared pointer to the ROS 2 node used for communication. */

  QString topic_name_ = ""; /**< Stores the current topic name for subscriptions. */

  std::map<std::string, std::string> color_map_; /**< Static map loaded from yaml. >*/
};

}  // namespace dua_rviz_plugins

#endif  // DUA_RVIZ_PLUGINS__TEXT_SUB_PANEL_HPP_
