/**
 * VisualTargetsDisplay header file.
 *
 * February 17, 2025
 */

/**
 * Copyright 2026 dotX Automation s.r.l.
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

#pragma once

#include <algorithm>
#include <deque>
#include <filesystem>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_map>

#include <dua_cv_bridge/dua_cv_bridge.hpp>

#include <opencv2/core.hpp>

#include <dua_qos_cpp/dua_qos.hpp>
#include <dua_mission_interfaces/msg/visual_targets.hpp>

#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/ros_topic_display.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <QDialog>
#include <QImage>
#include <QLabel>
#include <QMetaObject>
#include <QObject>
#include <QPixmap>
#include <QScrollArea>
#include <QVBoxLayout>
#include <QWidget>

#include <interactive_markers/interactive_marker_server.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace dua_rviz_plugins::displays::visual_targets
{

using geometry_msgs::msg::Pose;
using sensor_msgs::msg::CompressedImage;
using sensor_msgs::msg::Image;

using TargetInfo = std::tuple<std::string, Pose, Image>;
using TargetHistory = std::deque<TargetInfo>;

/**
 * @brief Stored data for a tracked visual target.
 */
struct TargetData
{
  std::string target_id;
  std::string class_id;
  std::string frame_id;
  Pose pose;
  TargetHistory history;
  rclcpp::Time last_seen;
};

/**
 * @brief Display visual targets in RViz.
 */
class VisualTargetsDisplay
  : public rviz_common::RosTopicDisplay<dua_mission_interfaces::msg::VisualTargets>
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
   * @brief Disable the display.
   */
  void onDisable() override;

  /**
   * @brief Reset the display.
   */
  void reset() override;

  /**
   * @brief Process the received message.
   */
  void processMessage(dua_mission_interfaces::msg::VisualTargets::ConstSharedPtr msg) override;

private Q_SLOTS:
  /**
   * @brief Apply a new maximum number of cached images.
   */
  void updateMaxImages();

  /**
   * @brief Apply the configured timeout and prune stale targets.
   */
  void updateTargetTimeout();

private:
  /**
   * @brief Build the internal storage key for a target.
   */
  std::string makeTargetKey(
    const std::string & class_id,
    const std::string & target_id) const;

  /**
   * @brief Remove stale targets according to the configured timeout.
   */
  void pruneStaleTargets(const rclcpp::Time & now);

  /**
   * @brief Rebuild all interactive markers from the stored targets.
   */
  void rebuildInteractiveMarkers();

  /**
   * @brief Create an interactive marker for a target.
   */
  void createInteractiveMarker(
    const std::string & target_key,
    const TargetData & target_data);

  /**
   * @brief Process the interactive marker feedback.
   */
  void processFeedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback,
    const std::string & target_key);

  /**
   * @brief Show the target image history in a dialog.
   */
  void showTargetImages(const std::string & target_key);

  rviz_common::properties::IntProperty * max_images_property_;
  rviz_common::properties::FloatProperty * target_timeout_property_;

  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  std::unordered_map<std::string, TargetData> targets_;
  std::mutex mutex_;
};

}  // namespace dua_rviz_plugins::displays::visual_targets
