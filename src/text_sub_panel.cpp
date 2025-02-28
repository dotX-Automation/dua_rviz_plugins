/**
 * TextSubPanel class source file.
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

#include <dua_rviz_plugins/text_sub_panel.hpp>

namespace dua_rviz_plugins
{

TextSubPanel::TextSubPanel(QWidget * parent)
: rviz_common::Panel(parent),
  topic_("/messages")
{
  // Create the input fields and labels
  QLabel * topic_label = new QLabel("Topic:");
  topic_input_ = new QLineEdit(topic_);
  connect(
    topic_input_,
    &QLineEdit::editingFinished,
    this,
    &TextSubPanel::update_topic);

  // Create the label for the message
  message_label_ = new QLabel(
    QString(
      "<font style='font-size:%2px;'>%3</font>")
    .arg(FONT_SIZE)
    .arg("Waiting..."));

  // Create the layout
  auto layout = new QVBoxLayout;
  // Add the widgets to the layout
  layout->addWidget(topic_label);
  layout->addWidget(topic_input_);
  layout->addWidget(message_label_);
  // Set the main layout
  setLayout(layout);
}

void TextSubPanel::onInitialize()
{
  // Call the base class implementation
  rviz_common::Panel::onInitialize();

  // Get the ROS node associated with RViz
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  // Load the color configuration from a YAML file
  std::string yaml_file = "src/dua_rviz_plugins/config/colors.yaml";
  YAML::Node config = YAML::LoadFile(yaml_file);
  for (auto it = config.begin(); it != config.end(); ++it) {
    std::string key = it->first.as<std::string>();
    std::string value = it->second.as<std::string>();
    color_map_[key] = value;
  }

  // Create the subscriber
  init_sub(topic_.toStdString());
}

void TextSubPanel::save(rviz_common::Config config) const
{
  // Call the base class implementation
  rviz_common::Panel::save(config);

  // Save the topic name
  config.mapSetValue("Topic", topic_);
}

void TextSubPanel::load(const rviz_common::Config & config)
{
  // Call the base class implementation
  rviz_common::Panel::load(config);

  // Load the topic name
  QString topic;
  if (config.mapGetString("Topic", &topic)) {
    topic_ = topic;
    topic_input_->setText(topic_);
    init_sub(topic_.toStdString());
  }
}

void TextSubPanel::init_sub(const std::string & topic_str)
{
  sub_ = node_->create_subscription<String>(
    topic_str,
    dua_qos::Reliable::get_datum_qos(),
    std::bind(&TextSubPanel::show_message_received,
      this,
      std::placeholders::_1));
}

void TextSubPanel::show_message_received(const std_msgs::msg::String::SharedPtr msg)
{
  // Set the default color
  std::string color = "#000000";  // Black

  // Check if the message type has a specific color
  auto it = color_map_.find(msg->data);
  if (it != color_map_.end()) {
    color = it->second;
  }

  // Check the received message
  std::string text;
  if (msg->data.empty()) {
    text = "Empty message";
  } else {
    text = msg->data;
  }

  // Update the label with the received message with color and text size
  message_label_->setText(
    QString(
      "<font color='%1' style='font-size:%2px;'>%3</font>")
    .arg(color.c_str())
    .arg(FONT_SIZE)
    .arg(QString::fromStdString(text)));
}

void TextSubPanel::update_topic()
{
  // Parse the topic name from the input field
  QString new_topic = topic_input_->text().trimmed();

  // Update the topic name and reset the subscriber
  if (!new_topic.isEmpty() && new_topic != topic_) {
    topic_ = new_topic;
    init_sub(topic_.toStdString());
  }
}

}  // namespace dua_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(dua_rviz_plugins::TextSubPanel, rviz_common::Panel)
