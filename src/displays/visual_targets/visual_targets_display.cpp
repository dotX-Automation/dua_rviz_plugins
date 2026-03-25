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

namespace dua_rviz_plugins::displays::visual_targets
{

VisualTargetsDisplay::VisualTargetsDisplay()
: rviz_common::RosTopicDisplay<dua_mission_interfaces::msg::VisualTargets>(),
  max_images_property_(nullptr),
  target_timeout_property_(nullptr)
{
  max_images_property_ = new rviz_common::properties::IntProperty(
    "Max Images",
    10,
    "Maximum number of images stored per visual target.",
    this,
    SLOT(updateMaxImages()));
  max_images_property_->setMin(1);
  max_images_property_->setMax(50);

  target_timeout_property_ = new rviz_common::properties::FloatProperty(
    "Target Timeout (s)",
    5.0,
    "How long to keep an unseen target before removing it.",
    this,
    SLOT(updateTargetTimeout()));
  target_timeout_property_->setMin(0.0);
  target_timeout_property_->setMax(60.0);
}

VisualTargetsDisplay::~VisualTargetsDisplay()
{
  if (initialized() && server_) {
    server_->clear();
    server_->applyChanges();
  }
}

void VisualTargetsDisplay::onInitialize()
{
  RosTopicDisplay::onInitialize();

  auto ros_node_abstraction = context_->getRosNodeAbstraction().lock();
  auto node = ros_node_abstraction->get_raw_node();

  server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
    "visual_targets",
    node);
}

void VisualTargetsDisplay::onDisable()
{
  RosTopicDisplay::onDisable();

  std::lock_guard<std::mutex> lock(mutex_);

  targets_.clear();
  if (server_) {
    server_->clear();
    server_->applyChanges();
  }
}

void VisualTargetsDisplay::reset()
{
  RosTopicDisplay::reset();

  std::lock_guard<std::mutex> lock(mutex_);

  targets_.clear();
  if (server_) {
    server_->clear();
    server_->applyChanges();
  }
}

void VisualTargetsDisplay::processMessage(
  dua_mission_interfaces::msg::VisualTargets::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  auto ros_node_abstraction = context_->getRosNodeAbstraction().lock();
  auto node = ros_node_abstraction->get_raw_node();
  const rclcpp::Time now = node->get_clock()->now();

  const auto max_images =
    static_cast<std::size_t>(std::max(1, max_images_property_->getInt()));

  const std::string & frame_id = msg->targets.header.frame_id;

  // Decompress the shared scene image once, since all targets refer to the same image.
  cv::Mat frame;
  Image::SharedPtr scene_image_msg = nullptr;
  std::string encoding(sensor_msgs::image_encodings::BGR8);

  if (!msg->image.format.empty()) {
    const std::size_t pos = msg->image.format.find(";");
    if (pos != std::string::npos) {
      encoding = msg->image.format.substr(0, pos);
    }
  }

  try {
    CompressedImage::ConstSharedPtr image_ptr =
      std::make_shared<CompressedImage>(msg->image);
    dua_cv_bridge::compressed_msg_to_frame(image_ptr, frame);
    scene_image_msg = dua_cv_bridge::frame_to_msg(frame, encoding);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("VisualTargetsDisplay"),
      "Error processing image: %s",
      e.what());
    return;
  }

  for (const auto & target : msg->targets.targets) {
    if (target.results.empty()) {
      continue;
    }

    const std::string & target_id = target.id;
    const std::string & class_id = target.results[0].hypothesis.class_id;
    const Pose & pose = target.results[0].pose.pose;

    TargetInfo target_info = {frame_id, pose, *scene_image_msg};

    auto target_it = targets_.find(target_id);
    if (target_it == targets_.end()) {
      TargetData target_data;
      target_data.class_id = class_id;
      target_data.frame_id = frame_id;
      target_data.pose = pose;
      target_data.history.push_front(target_info);
      target_data.last_seen = now;
      targets_.emplace(target_id, std::move(target_data));
    } else {
      TargetData & target_data = target_it->second;
      target_data.class_id = class_id;
      target_data.frame_id = frame_id;
      target_data.pose = pose;
      target_data.history.push_front(target_info);
      target_data.last_seen = now;

      while (target_data.history.size() > max_images) {
        target_data.history.pop_back();
      }
    }

    auto & history = targets_[target_id].history;
    while (history.size() > max_images) {
      history.pop_back();
    }
  }

  pruneStaleTargets(now);
  rebuildInteractiveMarkers();
}

void VisualTargetsDisplay::updateMaxImages()
{
  std::lock_guard<std::mutex> lock(mutex_);

  const auto max_images =
    static_cast<std::size_t>(std::max(1, max_images_property_->getInt()));

  for (auto & [target_id, target_data] : targets_) {
    while (target_data.history.size() > max_images) {
      target_data.history.pop_back();
    }
  }
}

void VisualTargetsDisplay::updateTargetTimeout()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!server_) {
    return;
  }

  auto ros_node_abstraction = context_->getRosNodeAbstraction().lock();
  auto node = ros_node_abstraction->get_raw_node();
  const rclcpp::Time now = node->get_clock()->now();

  pruneStaleTargets(now);
  rebuildInteractiveMarkers();
}

void VisualTargetsDisplay::pruneStaleTargets(const rclcpp::Time & now)
{
  const double timeout = std::max(0.0f, target_timeout_property_->getFloat());

  for (auto it = targets_.begin(); it != targets_.end(); ) {
    const double age = (now - it->second.last_seen).seconds();
    if (age > timeout) {
      it = targets_.erase(it);
    } else {
      ++it;
    }
  }
}

void VisualTargetsDisplay::rebuildInteractiveMarkers()
{
  if (!server_) {
    return;
  }

  server_->clear();

  for (const auto & [target_id, target_data] : targets_) {
    createInteractiveMarker(target_id, target_data);
  }

  server_->applyChanges();
}

void VisualTargetsDisplay::createInteractiveMarker(
  const std::string & target_id,
  const TargetData & target_data)
{
  const Pose & pose = target_data.pose;
  const std::string & class_id = target_data.class_id;

  visualization_msgs::msg::Marker marker;
  marker.header.set__stamp(rclcpp::Time(0));
  marker.header.set__frame_id("map");
  marker.set__action(visualization_msgs::msg::Marker::ADD);
  marker.pose.set__position(pose.position);
  marker.pose.set__orientation(pose.orientation);

  if (class_id.find("ArUco") != std::string::npos) {
    marker.set__type(visualization_msgs::msg::Marker::SPHERE);
    marker.scale.set__x(1.0);
    marker.scale.set__y(1.0);
    marker.scale.set__z(1.0);
    marker.color.set__r(1.0);
    marker.color.set__g(0.0);
    marker.color.set__b(0.0);
    marker.color.set__a(1.0);
  } else if (class_id.find("QR") != std::string::npos) {
    marker.set__type(visualization_msgs::msg::Marker::SPHERE);
    marker.scale.set__x(1.0);
    marker.scale.set__y(1.0);
    marker.scale.set__z(1.0);
    marker.color.set__r(0.0);
    marker.color.set__g(0.0);
    marker.color.set__b(1.0);
    marker.color.set__a(1.0);
  } else {
    std::string sanitized_class_id = class_id;
    std::replace(sanitized_class_id.begin(), sanitized_class_id.end(), ' ', '_');

    const std::string base_path =
      (std::filesystem::path(__FILE__).parent_path() / "../../../dae/").string();
    const std::string mesh_resource = "file:///" + base_path + sanitized_class_id + ".dae";
    const std::string local_path = mesh_resource.substr(7);

    if (std::filesystem::exists(local_path)) {
      marker.set__type(visualization_msgs::msg::Marker::MESH_RESOURCE);
      marker.scale.set__x(1.0);
      marker.scale.set__y(1.0);
      marker.scale.set__z(1.0);
      marker.color.set__r(1.0);
      marker.color.set__g(1.0);
      marker.color.set__b(1.0);
      marker.color.set__a(1.0);
      marker.set__mesh_resource(mesh_resource);
      marker.set__mesh_use_embedded_materials(true);
    } else {
      marker.set__type(visualization_msgs::msg::Marker::SPHERE);
      marker.scale.set__x(1.0);
      marker.scale.set__y(1.0);
      marker.scale.set__z(1.0);
      marker.color.set__r(0.5);
      marker.color.set__g(0.5);
      marker.color.set__b(0.5);
      marker.color.set__a(1.0);
    }
  }

  visualization_msgs::msg::InteractiveMarkerControl marker_control;
  marker_control.set__name(target_id);
  marker_control.set__orientation_mode(
    visualization_msgs::msg::InteractiveMarkerControl::FIXED);
  marker_control.set__interaction_mode(
    visualization_msgs::msg::InteractiveMarkerControl::BUTTON);
  marker_control.set__always_visible(true);
  marker_control.markers.push_back(marker);

  visualization_msgs::msg::InteractiveMarker interactive_marker;
  interactive_marker.header.set__stamp(rclcpp::Time(0));
  interactive_marker.header.set__frame_id("map");
  interactive_marker.pose.set__position(pose.position);
  interactive_marker.pose.set__orientation(pose.orientation);
  interactive_marker.set__name(target_id);
  interactive_marker.set__description(class_id);
  interactive_marker.set__scale(0.75);
  interactive_marker.controls.push_back(marker_control);

  server_->insert(
    interactive_marker,
    std::bind(
      &VisualTargetsDisplay::processFeedback,
      this,
      std::placeholders::_1,
      target_id));
}

void VisualTargetsDisplay::processFeedback(
  const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback,
  const std::string & target_id)
{
  if (feedback->event_type ==
    visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_DOWN)
  {
    QMetaObject::invokeMethod(
      this,
      [this, target_id]() {showTargetImages(target_id);},
      Qt::QueuedConnection);
  }
}

void VisualTargetsDisplay::showTargetImages(const std::string & target_id)
{
  std::lock_guard<std::mutex> lock(mutex_);

  const auto target_it = targets_.find(target_id);
  if (target_it == targets_.end() || target_it->second.history.empty()) {
    return;
  }

  const TargetData & target_data = target_it->second;
  const TargetHistory & history = target_data.history;

  QDialog * dialog = new QDialog();
  dialog->setAttribute(Qt::WA_DeleteOnClose);

  std::string title = target_data.class_id;
  std::replace(title.begin(), title.end(), '_', ' ');
  dialog->setWindowTitle(QString::fromStdString(title));

  QScrollArea * scroll_area = new QScrollArea(dialog);
  QWidget * scroll_content = new QWidget(scroll_area);
  QVBoxLayout * scroll_layout = new QVBoxLayout(scroll_content);
  scroll_layout->setContentsMargins(0, 0, 0, 0);

  for (const auto & target_info : history) {
    const std::string & frame_id = std::get<0>(target_info);
    const sensor_msgs::msg::Image & image = std::get<2>(target_info);

    QImage qimage;
    const auto & encoding = image.encoding;

    if (encoding == sensor_msgs::image_encodings::RGB8) {
      qimage = QImage(
        image.data.data(),
        image.width,
        image.height,
        QImage::Format_RGB888);
    } else if (encoding == sensor_msgs::image_encodings::RGBA8) {
      qimage = QImage(
        image.data.data(),
        image.width,
        image.height,
        QImage::Format_RGBA8888);
    } else if (encoding == sensor_msgs::image_encodings::MONO8) {
      qimage = QImage(
        image.data.data(),
        image.width,
        image.height,
        QImage::Format_Grayscale8);
    } else if (encoding == sensor_msgs::image_encodings::BGR8) {
      QImage temp(
        image.data.data(),
        image.width,
        image.height,
        QImage::Format_RGB888);
      qimage = temp.rgbSwapped();
    } else if (encoding == sensor_msgs::image_encodings::BGRA8) {
      QImage temp(
        image.data.data(),
        image.width,
        image.height,
        QImage::Format_RGBA8888);
      qimage = temp.rgbSwapped();
    } else {
      continue;
    }

    qimage = qimage.copy();

    QVBoxLayout * entry_layout = new QVBoxLayout();
    entry_layout->setAlignment(Qt::AlignTop);

    QLabel * frame_label = new QLabel(QString::fromStdString(frame_id));
    frame_label->setAlignment(Qt::AlignCenter);
    frame_label->setStyleSheet("font-weight: bold; margin: 2px;");
    entry_layout->addWidget(frame_label);

    QLabel * image_label = new QLabel();
    image_label->setPixmap(
      QPixmap::fromImage(qimage).scaledToWidth(640, Qt::SmoothTransformation));
    image_label->setAlignment(Qt::AlignCenter);
    entry_layout->addWidget(image_label);

    QWidget * entry_widget = new QWidget();
    entry_widget->setLayout(entry_layout);
    scroll_layout->addWidget(entry_widget);
  }

  scroll_content->setLayout(scroll_layout);
  scroll_area->setWidget(scroll_content);
  scroll_area->setWidgetResizable(true);

  QVBoxLayout * dialog_layout = new QVBoxLayout(dialog);
  dialog_layout->addWidget(scroll_area);
  dialog->setLayout(dialog_layout);

  dialog->setMinimumSize(640, 480);
  dialog->adjustSize();
  dialog->show();
}

}  // namespace dua_rviz_plugins::displays::visual_targets

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  dua_rviz_plugins::displays::visual_targets::VisualTargetsDisplay,
  rviz_common::Display)
