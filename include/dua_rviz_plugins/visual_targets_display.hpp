/**
 * VisualTargetsDisplay header file.
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

#ifndef DUA_RVIZ_PLUGINS__VISUAL_TARGETS_DISPLAY_HPP_
#define DUA_RVIZ_PLUGINS__VISUAL_TARGETS_DISPLAY_HPP_

#include <dua_qos_cpp/dua_qos.hpp>
#include <dua_interfaces/msg/visual_targets.hpp>

#include <rviz_common/ros_topic_display.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <tuple>

#include <QDialog>
#include <QImage>
#include <QLabel>
#include <QObject>
#include <QApplication>
#include <QMetaObject>
#include <QVBoxLayout>

#include <interactive_markers/interactive_marker_server.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>

namespace dua_rviz_plugins
{

using geometry_msgs::msg::Pose;
using sensor_msgs::msg::Image;

typedef std::tuple<std::string, Pose, Image> Info;
typedef std::vector<Info> Infos;

/**
 * @brief Display visual targets in RViz.
 */
class VisualTargetsDisplay
  : public rviz_common::RosTopicDisplay<dua_interfaces::msg::VisualTargets>
{
  Q_OBJECT

public:
  /**
   * @brief Constructor.
   */
  VisualTargetsDisplay();
  /**
   * @brief Destructor.
   */
  ~VisualTargetsDisplay() override;

protected:
  /**
   * @brief Initialize the display.
   */
  void onInitialize() override;
  /**
   * @brief Process the received message.
   */
  void processMessage(dua_interfaces::msg::VisualTargets::ConstSharedPtr msg) override;

private:
  /**
   * @brief Create an interactive marker.
   */
  void createInteractiveMarker(
    const geometry_msgs::msg::Pose & pose,
    const std::string & id);
  /**
   * @brief Process the interactive marker feedback.
   */
  void processFeedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback,
    const std::string & id);
  /**
   * @brief Show the image in a dialog.
   */
  void showImage(const std::string & id);

  std::string frame_id_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  std::unordered_map<std::string, Infos> map_;
  std::mutex mutex_;
};

}  // namespace dua_rviz_plugins

#endif  // DUA_RVIZ_PLUGINS__VISUAL_TARGETS_DISPLAY_HPP_
