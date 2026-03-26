/**
 * VisualTargetsDisplay class source file.
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

#include <dua_rviz_plugins/displays/visual_targets/visual_targets_display.hpp>

#include <algorithm>
#include <functional>
#include <sstream>
#include <utility>

#include <pluginlib/class_list_macros.hpp>

namespace dua_rviz_plugins::displays::visual_targets
{

VisualTargetsDisplay::VisualTargetsDisplay()
: rviz_common::RosTopicDisplay<VisualTargets>(),
  max_images_prop_(nullptr),
  target_timeout_prop_(nullptr)
{
  max_images_prop_ = new rviz_common::properties::IntProperty(
    "Max Images",
    10,
    "Maximum number of images stored per visual target.",
    this,
    SLOT(updateMaxImages()));
  max_images_prop_->setMin(1);
  max_images_prop_->setMax(10);

  target_timeout_prop_ = new rviz_common::properties::FloatProperty(
    "Target Timeout (s)",
    5.0,
    "How long to keep an unseen target before removing it.",
    this,
    SLOT(updateTargetTimeout()));
  target_timeout_prop_->setMin(0.0);
  target_timeout_prop_->setMax(60.0);
}

VisualTargetsDisplay::~VisualTargetsDisplay()
{
  if (server_) {
    server_->clear();
    server_->applyChanges();
  }
}

void VisualTargetsDisplay::onInitialize()
{
  RosTopicDisplay::onInitialize();

  auto ros_node_abstraction = context_->getRosNodeAbstraction().lock();
  if (!ros_node_abstraction) {
    setStatus(
      rviz_common::properties::StatusProperty::Error,
      "ROS Node",
      "Failed to access ROS node abstraction.");
    return;
  }

  auto node = ros_node_abstraction->get_raw_node();
  server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
    "visual_targets",
    node);
}

void VisualTargetsDisplay::onDisable()
{
  RosTopicDisplay::onDisable();

  {
    std::lock_guard<std::mutex> lock(mutex_);
    targets_.clear();
  }

  if (server_) {
    server_->clear();
    server_->applyChanges();
  }
}

void VisualTargetsDisplay::reset()
{
  RosTopicDisplay::reset();

  {
    std::lock_guard<std::mutex> lock(mutex_);
    targets_.clear();
  }

  if (server_) {
    server_->clear();
    server_->applyChanges();
  }
}

void VisualTargetsDisplay::processMessage(VisualTargets::ConstSharedPtr msg)
{
  if (!msg || !server_) {
    return;
  }

  auto ros_node_abstraction = context_->getRosNodeAbstraction().lock();
  if (!ros_node_abstraction) {
    setStatus(
      rviz_common::properties::StatusProperty::Error,
      "ROS Node",
      "Failed to access ROS node abstraction.");
    return;
  }

  auto node = ros_node_abstraction->get_raw_node();
  const rclcpp::Time now = node->get_clock()->now();

  cv::Mat frame;
  std::string encoding(sensor_msgs::image_encodings::BGR8);

  if (!msg->image.format.empty()) {
    const std::size_t pos = msg->image.format.find(';');
    if (pos != std::string::npos && pos > 0U) {
      encoding = msg->image.format.substr(0, pos);
    }
  }

  std::shared_ptr<Image> scene_image_msg;
  try {
    auto image_ptr = std::make_shared<CompressedImage>(msg->image);
    dua_cv_bridge::compressed_msg_to_frame(image_ptr, frame);
    scene_image_msg = dua_cv_bridge::frame_to_msg(frame, encoding);
  } catch (const std::exception & e) {
    setStatus(
      rviz_common::properties::StatusProperty::Error,
      "Image",
      QString("Failed to decode compressed image: %1").arg(e.what()));
    return;
  }

  TargetList targets_snapshot;

  {
    std::lock_guard<std::mutex> lock(mutex_);

    const auto max_images = maxImages();

    for (const auto & target : msg->targets.targets) {
      if (target.id.empty()) {
        RCLCPP_WARN(
          rclcpp::get_logger("VisualTargetsDisplay"),
          "Skipping target with empty id.");
        continue;
      }

      if (target.results.empty()) {
        continue;
      }

      const auto & result = target.results.front();
      const std::string & target_id = target.id;
      const std::string & class_id = result.hypothesis.class_id;
      const std::string frame_id = msg->targets.header.frame_id;
      const Pose & pose = result.pose.pose;

      const std::string target_key = makeTargetKey(target_id, class_id);
      auto [it, inserted] = targets_.try_emplace(
        target_key,
        TargetData{
          target_id,
          class_id,
          frame_id,
          pose,
          {},
          now});

      (void)inserted;

      TargetData & target_data = it->second;
      target_data.target_id = target_id;
      target_data.class_id = class_id;
      target_data.frame_id = frame_id;
      target_data.pose = pose;
      target_data.last_seen = now;

      target_data.history.push_front(
        TargetSnapshot{
          frame_id,
          pose,
          now,
          scene_image_msg});

      while (target_data.history.size() > max_images) {
        target_data.history.pop_back();
      }
    }

    pruneStaleTargetsLocked(now);
    targets_snapshot = snapshotTargetsLocked();
  }

  rebuildInteractiveMarkers(targets_snapshot);
  setStatus(
    rviz_common::properties::StatusProperty::Ok,
    "Targets",
    QString("Tracking %1 target(s).").arg(static_cast<int>(targets_snapshot.size())));
}

void VisualTargetsDisplay::updateMaxImages()
{
  std::lock_guard<std::mutex> lock(mutex_);

  const auto max_images = maxImages();
  for (auto & [target_key, target_data] : targets_) {
    (void)target_key;
    while (target_data.history.size() > max_images) {
      target_data.history.pop_back();
    }
  }
}

void VisualTargetsDisplay::updateTargetTimeout()
{
  if (!server_) {
    return;
  }

  auto ros_node_abstraction = context_->getRosNodeAbstraction().lock();
  if (!ros_node_abstraction) {
    return;
  }

  auto node = ros_node_abstraction->get_raw_node();
  const rclcpp::Time now = node->get_clock()->now();

  TargetList targets_snapshot;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pruneStaleTargetsLocked(now);
    targets_snapshot = snapshotTargetsLocked();
  }

  rebuildInteractiveMarkers(targets_snapshot);
}

void VisualTargetsDisplay::pruneStaleTargetsLocked(const rclcpp::Time & now)
{
  const double timeout = targetTimeoutSeconds();

  for (auto it = targets_.begin(); it != targets_.end(); ) {
    const double age = (now - it->second.last_seen).seconds();
    if (age > timeout) {
      it = targets_.erase(it);
    } else {
      ++it;
    }
  }
}

VisualTargetsDisplay::TargetList VisualTargetsDisplay::snapshotTargetsLocked() const
{
  TargetList result;
  result.reserve(targets_.size());

  for (const auto & item : targets_) {
    result.push_back(item);
  }

  return result;
}

void VisualTargetsDisplay::rebuildInteractiveMarkers(const TargetList & targets)
{
  if (!server_) {
    return;
  }

  server_->clear();

  for (const auto & [target_key, target_data] : targets) {
    createInteractiveMarker(target_key, target_data);
  }

  server_->applyChanges();
}

void VisualTargetsDisplay::createInteractiveMarker(
  const std::string & target_key,
  const TargetData & target_data)
{
  if (target_data.frame_id.empty()) {
    RCLCPP_WARN(
      rclcpp::get_logger("VisualTargetsDisplay"),
      "Skipping marker creation for target '%s' because frame_id is empty.",
      target_key.c_str());
    return;
  }

  visualization_msgs::msg::Marker marker = makeMarker(target_data);

  visualization_msgs::msg::InteractiveMarkerControl marker_control;
  marker_control.name = target_key;
  marker_control.orientation_mode =
    visualization_msgs::msg::InteractiveMarkerControl::FIXED;
  marker_control.interaction_mode =
    visualization_msgs::msg::InteractiveMarkerControl::BUTTON;
  marker_control.always_visible = true;
  marker_control.markers.push_back(marker);

  visualization_msgs::msg::InteractiveMarker interactive_marker;
  interactive_marker.header.stamp = rclcpp::Time(0);
  interactive_marker.header.frame_id = target_data.frame_id;
  interactive_marker.pose = target_data.pose;
  interactive_marker.name = target_key;

  std::ostringstream description;
  description << target_data.class_id << " [" << target_data.target_id << "]";
  interactive_marker.description = description.str();
  interactive_marker.scale = 0.75;
  interactive_marker.controls.push_back(marker_control);

  server_->insert(
    interactive_marker,
    std::bind(
      &VisualTargetsDisplay::processFeedback,
      this,
      std::placeholders::_1,
      target_key));
}

visualization_msgs::msg::Marker VisualTargetsDisplay::makeMarker(
  const TargetData & target_data) const
{
  visualization_msgs::msg::Marker marker;
  marker.header.stamp = rclcpp::Time(0);
  marker.header.frame_id = target_data.frame_id;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose = target_data.pose;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.color.a = 1.0;

  const std::string & class_id = target_data.class_id;

  if (class_id.find("ArUco") != std::string::npos) {
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.color.r = 1.0F;
    marker.color.g = 0.0F;
    marker.color.b = 0.0F;
    return marker;
  }

  if (class_id.find("QR") != std::string::npos) {
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.color.r = 0.0F;
    marker.color.g = 0.0F;
    marker.color.b = 1.0F;
    return marker;
  }

  if (const auto mesh_resource = resolveMeshResource(class_id)) {
    marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    marker.mesh_resource = *mesh_resource;
    marker.mesh_use_embedded_materials = true;
    marker.color.r = 1.0F;
    marker.color.g = 1.0F;
    marker.color.b = 1.0F;
    return marker;
  }

  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.color.r = 0.5F;
  marker.color.g = 0.5F;
  marker.color.b = 0.5F;
  return marker;
}

std::optional<std::string> VisualTargetsDisplay::resolveMeshResource(
  const std::string & class_id) const
{
  std::string sanitized_class_id = class_id;
  std::replace(sanitized_class_id.begin(), sanitized_class_id.end(), ' ', '_');

  const std::string base_path =
    (std::filesystem::path(__FILE__).parent_path() / "../../../dae/").string();
  const std::string mesh_resource =
    "file:///" + base_path + sanitized_class_id + ".dae";
  const std::string local_path = mesh_resource.substr(7);

  if (std::filesystem::exists(local_path)) {
    return mesh_resource;
  }

  return std::nullopt;
}

void VisualTargetsDisplay::processFeedback(
  const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback,
  const std::string & target_key)
{
  if (!feedback) {
    return;
  }

  if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_DOWN) {
    QMetaObject::invokeMethod(
      this,
      [this, target_key]() {showTargetImages(target_key);},
      Qt::QueuedConnection);
  }
}

void VisualTargetsDisplay::showTargetImages(const std::string & target_key)
{
  TargetData target_data;

  {
    std::lock_guard<std::mutex> lock(mutex_);

    const auto it = targets_.find(target_key);
    if (it == targets_.end() || it->second.history.empty()) {
      return;
    }

    target_data = it->second;
  }

  auto * dialog = new QDialog();
  dialog->setAttribute(Qt::WA_DeleteOnClose);

  std::string title = target_data.class_id;
  std::replace(title.begin(), title.end(), '_', ' ');
  title += " [" + target_data.target_id + "]";
  dialog->setWindowTitle(QString::fromStdString(title));

  auto * scroll_area = new QScrollArea(dialog);
  auto * scroll_content = new QWidget(scroll_area);
  auto * scroll_layout = new QVBoxLayout(scroll_content);
  scroll_layout->setContentsMargins(6, 6, 6, 6);
  scroll_layout->setSpacing(10);

  for (const auto & snapshot : target_data.history) {
    if (!snapshot.image) {
      continue;
    }

    QImage qimage = imageMsgToQImage(*snapshot.image);
    if (qimage.isNull()) {
      continue;
    }

    auto * entry_layout = new QVBoxLayout();
    entry_layout->setAlignment(Qt::AlignTop);
    entry_layout->setSpacing(4);

    const auto stamp_sec = snapshot.stamp.seconds();

    std::ostringstream header_text;
    header_text << snapshot.frame_id << " | t=" << stamp_sec;

    auto * frame_label = new QLabel(QString::fromStdString(header_text.str()));
    frame_label->setAlignment(Qt::AlignCenter);
    frame_label->setStyleSheet("font-weight: bold; margin: 2px;");
    entry_layout->addWidget(frame_label);

    auto * image_label = new QLabel();
    image_label->setPixmap(
      QPixmap::fromImage(qimage).scaledToWidth(640, Qt::SmoothTransformation));
    image_label->setAlignment(Qt::AlignCenter);
    entry_layout->addWidget(image_label);

    auto * entry_widget = new QWidget();
    entry_widget->setLayout(entry_layout);
    scroll_layout->addWidget(entry_widget);
  }

  scroll_content->setLayout(scroll_layout);
  scroll_area->setWidget(scroll_content);
  scroll_area->setWidgetResizable(true);

  auto * dialog_layout = new QVBoxLayout(dialog);
  dialog_layout->addWidget(scroll_area);
  dialog->setLayout(dialog_layout);

  dialog->setMinimumSize(680, 500);
  dialog->show();
}

QImage VisualTargetsDisplay::imageMsgToQImage(const Image & image)
{
  const auto width = static_cast<int>(image.width);
  const auto height = static_cast<int>(image.height);

  if (width <= 0 || height <= 0 || image.data.empty()) {
    return {};
  }

  const auto & encoding = image.encoding;

  if (encoding == sensor_msgs::image_encodings::RGB8) {
    return QImage(
      image.data.data(),
      width,
      height,
      QImage::Format_RGB888).copy();
  }

  if (encoding == sensor_msgs::image_encodings::RGBA8) {
    return QImage(
      image.data.data(),
      width,
      height,
      QImage::Format_RGBA8888).copy();
  }

  if (encoding == sensor_msgs::image_encodings::MONO8) {
    return QImage(
      image.data.data(),
      width,
      height,
      QImage::Format_Grayscale8).copy();
  }

  if (encoding == sensor_msgs::image_encodings::BGR8) {
    QImage temp(
      image.data.data(),
      width,
      height,
      QImage::Format_RGB888);
    return temp.rgbSwapped().copy();
  }

  if (encoding == sensor_msgs::image_encodings::BGRA8) {
    QImage temp(
      image.data.data(),
      width,
      height,
      QImage::Format_RGBA8888);
    return temp.rgbSwapped().copy();
  }

  return {};
}

}  // namespace dua_rviz_plugins::displays::visual_targets

PLUGINLIB_EXPORT_CLASS(
  dua_rviz_plugins::displays::visual_targets::VisualTargetsDisplay,
  rviz_common::Display)
