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

#include <deque>
#include <filesystem>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <dua_cv_bridge/dua_cv_bridge.hpp>

#include <opencv2/core.hpp>

#include <dua_mission_interfaces/msg/visual_targets.hpp>

#include <geometry_msgs/msg/pose.hpp>

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

using dua_mission_interfaces::msg::VisualTargets;
using geometry_msgs::msg::Pose;
using sensor_msgs::msg::CompressedImage;
using sensor_msgs::msg::Image;

/**
 * @brief One historical observation of a target.
 *
 * The image pointer is shared so that a single scene snapshot can be reused
 * across multiple targets detected in the same incoming message.
 */
struct TargetSnapshot
{
  std::string frame_id;
  Pose pose;
  rclcpp::Time stamp;
  std::shared_ptr<Image> image;
};

using TargetHistory = std::deque<TargetSnapshot>;

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
  : public rviz_common::RosTopicDisplay<VisualTargets>
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
   * @brief Update the display at each frame.
   */
  void update(float wall_dt, float ros_dt) override;

  /**
   * @brief Process the received message.
   */
  void processMessage(VisualTargets::ConstSharedPtr msg) override;

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
  using TargetMap = std::unordered_map<std::string, TargetData>;
  using TargetList = std::vector<std::pair<std::string, TargetData>>;

  /**
   * @brief Build the internal storage key for a target.
   */
  std::string makeTargetKey(
    const std::string & target_id,
    const std::string & class_id) const
  {
    return class_id + "::" + target_id;
  }

  /**
   * @brief Remove stale targets according to the configured timeout.
   */
  void pruneStaleTargetsLocked(const rclcpp::Time & now);

  /**
   * @brief Return a copy of all targets for lock-free marker rebuilding.
   */
  TargetList snapshotTargetsLocked() const;

  /**
   * @brief Rebuild all interactive markers from a target snapshot.
   */
  void rebuildInteractiveMarkers(const TargetList & targets);

  /**
   * @brief Create an interactive marker for a target.
   */
  void createInteractiveMarker(
    const std::string & target_key,
    const TargetData & target_data);

  /**
   * @brief Build a visualization marker for a target.
   */
  visualization_msgs::msg::Marker makeMarker(
    const TargetData & target_data) const;

  /**
   * @brief Resolve a mesh resource for a class id, if available.
   */
  std::optional<std::string> resolveMeshResource(
    const std::string & class_id) const;

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

  /**
   * @brief Convert a ROS image message into QImage.
   */
  static QImage imageMsgToQImage(const Image & image);

  /**
   * @brief Filter target position with exponential smoothing.
   */
  geometry_msgs::msg::Point filterPosition(
    const geometry_msgs::msg::Point & previous,
    const geometry_msgs::msg::Point & current) const;

  /**
   * @brief Return whether the position filter should be reset.
   */
  bool shouldResetPositionFilter(
    const TargetData & target_data,
    const rclcpp::Time & now) const;


  rviz_common::properties::IntProperty * max_images_prop_;
  std::size_t maxImages() const
  {
    return static_cast<std::size_t>(std::max(1, max_images_prop_->getInt()));
  }

  rviz_common::properties::FloatProperty * target_timeout_prop_;
  double targetTimeoutSeconds() const
  {
    return std::max(0.0, static_cast<double>(target_timeout_prop_->getFloat()));
  }

  rviz_common::properties::FloatProperty * position_filter_alpha_prop_;
  double positionFilterAlpha() const
  {
    return std::clamp(static_cast<double>(position_filter_alpha_prop_->getFloat()), 0.0, 1.0);
  }

  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  TargetMap targets_;
  mutable std::mutex mutex_;
};

}  // namespace dua_rviz_plugins::displays::visual_targets
