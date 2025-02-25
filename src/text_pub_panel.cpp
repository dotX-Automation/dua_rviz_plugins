/**
 * Implementation of RViz2 TextPubPanel for DUA modules.
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

#include <dua_rviz_plugins/text_pub_panel.hpp>

namespace dua_rviz_plugins
{

TextPubPanel::TextPubPanel(QWidget * parent)
: rviz_common::Panel(parent),
  topic_name_("/messages")
{
  // Create GUI elements
  QVBoxLayout * layout = new QVBoxLayout;

  // Create the input fields and buttons
  topic_label_ = new QLabel("Topic:");
  topic_input_ = new QLineEdit(topic_name_);
  connect(topic_input_, &QLineEdit::editingFinished, this, &TextPubPanel::updateTopic);

  // Create the message input field
  message_input_ = new QLineEdit();
  message_input_->setPlaceholderText("Enter message");

  // Create the send button
  send_button_ = new QPushButton("PUBLISH");
  connect(send_button_, &QPushButton::clicked, this, &TextPubPanel::sendMessage);
  send_button_->setStyleSheet("color: black; font-weight: bold;");

  // Create the status label
  status_label_ = new QLabel("Ready to publish");

  // Add the elements to the layout
  layout->addWidget(topic_label_);
  layout->addWidget(topic_input_);
  layout->addWidget(new QLabel("Message:"));
  layout->addWidget(message_input_);
  layout->addWidget(send_button_);
  layout->addWidget(status_label_);
  setLayout(layout);
}

void TextPubPanel::onInitialize()
{
  // Call the base class implementation
  rviz_common::Panel::onInitialize();

  // Get the ROS node associated with RViz
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
}

void TextPubPanel::updateTopic()
{
  // Get the new topic name
  QString new_topic = topic_input_->text().trimmed();

  // Update the topic name and reset the publisher
  if (!new_topic.isEmpty() && new_topic != topic_name_) {
    topic_name_ = new_topic;
    publisher_.reset();
  }
}

void TextPubPanel::sendMessage()
{
  // Check if the node is valid
  if (!node_) return;

  // Create a new publisher if necessary
  if (!publisher_) {
    publisher_ = node_->create_publisher<std_msgs::msg::String>(topic_name_.toStdString(), 10);
  }

  // Publish the message
  auto msg = std_msgs::msg::String();
  msg.data = message_input_->text().toStdString();
  if (!msg.data.empty()) {
    publisher_->publish(msg);
    status_label_->setText("Message sent to " + topic_name_);
  }
}

}  // namespace dua_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(dua_rviz_plugins::TextPubPanel, rviz_common::Panel)
