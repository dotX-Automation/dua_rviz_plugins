/**
 * TextPubPanel class source file.
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

#include <dua_rviz_plugins/text_pub_panel.hpp>

namespace dua_rviz_plugins
{

TextPubPanel::TextPubPanel(QWidget * parent)
: rviz_common::Panel(parent),
  topic_("/messages")
{
  // Create the input fields and buttons
  QLabel * topic_label = new QLabel("Topic:");
  topic_input_ = new QLineEdit(topic_);
  connect(
    topic_input_,
    &QLineEdit::editingFinished,
    this,
    &TextPubPanel::update_topic);

  // Create the message input field
  QLabel * message_label = new QLabel("Message:");
  message_input_ = new QLineEdit();
  message_input_->setPlaceholderText("Enter message");

  // Create the send button
  QPushButton * send_button = new QPushButton("PUBLISH");
  send_button->setStyleSheet("color: black; font-weight: bold;");
  connect(
    send_button,
    &QPushButton::clicked,
    this,
    &TextPubPanel::send_button_clicked);

  // Create the layout
  auto layout = new QVBoxLayout();
  // Add the widgets to the layout
  layout->addWidget(topic_label);
  layout->addWidget(topic_input_);
  layout->addWidget(message_label);
  layout->addWidget(message_input_);
  layout->addWidget(send_button);
  // Set the main layout
  setLayout(layout);
}

void TextPubPanel::onInitialize()
{
  // Call the base class implementation
  rviz_common::Panel::onInitialize();

  // Get the ROS node associated with RViz
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  // Create the publisher
  init_pub(topic_.toStdString());
}

void TextPubPanel::save(rviz_common::Config config) const
{
  // Call the base class implementation
  rviz_common::Panel::save(config);

  // Save the topic name
  config.mapSetValue("Topic", topic_);
}

void TextPubPanel::load(const rviz_common::Config & config)
{
  // Call the base class implementation
  rviz_common::Panel::load(config);

  // Load the topic name
  QString topic;
  if (config.mapGetString("Topic", &topic)) {
    topic_ = topic;
    topic_input_->setText(topic_);
    init_pub(topic_.toStdString());
  }
}

void TextPubPanel::init_pub(const std::string & topic_str)
{
  pub_ = node_->create_publisher<String>(
    topic_str,
    dua_qos::Reliable::get_datum_qos());
}

void TextPubPanel::send_button_clicked()
{
  // Parse the message from the input field
  String msg;
  msg.data = message_input_->text().toStdString();
  // Publish the message REPUBLISH_COUNT times
  for (int i = 0; i < REPUBLISH_COUNT; i++) {
    pub_->publish(msg);
  }
}

void TextPubPanel::update_topic()
{
  // Parse the topic name from the input field
  QString new_topic = topic_input_->text().trimmed();

  // Update the topic name and reset the publisher
  if (!new_topic.isEmpty() && new_topic != topic_) {
    topic_ = new_topic;
    init_pub(topic_.toStdString());
  }
}

}  // namespace dua_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(dua_rviz_plugins::TextPubPanel, rviz_common::Panel)
