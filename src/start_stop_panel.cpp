/**
 * Implementation of RViz2 StartStopPanel for DUA modules.
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

#include <dua_rviz_plugins/start_stop_panel.hpp>

namespace dua_rviz_plugins
{

StartStopPanel::StartStopPanel(QWidget * parent)
: rviz_common::Panel(parent),
  topic_start_name_("/start"),
  topic_stop_name_("/stop")
{
  // Initialize ROS2 node
  node_ = std::make_shared<rclcpp::Node>("rviz_button_node");

  // Create publishers for two different topics
  publisher_start_ = node_->create_publisher<std_msgs::msg::Empty>(
    topic_start_name_.toStdString(),
    dua_qos::Reliable::get_datum_qos());
  publisher_stop_ = node_->create_publisher<std_msgs::msg::Empty>(
    topic_stop_name_.toStdString(),
    dua_qos::Reliable::get_datum_qos());

  // Create the UI elements for start topic
  QLabel* label_start = new QLabel("Start Topic:", this);
  topic_start_input_ = new QLineEdit(topic_start_name_, this);
  connect(topic_start_input_, &QLineEdit::editingFinished, this, &StartStopPanel::updateStartTopic);
  button_start_ = new QPushButton("START", this);
  connect(button_start_, &QPushButton::clicked, this, &StartStopPanel::startButtonClicked);
  button_start_->setStyleSheet("color: green; font-weight: bold;");

  // Create the UI elements for stop topic
  QLabel* label_stop = new QLabel("Stop Topic:", this);
  topic_stop_input_ = new QLineEdit(topic_stop_name_, this);
  connect(topic_stop_input_, &QLineEdit::editingFinished, this, &StartStopPanel::updateStopTopic);
  button_stop_ = new QPushButton("STOP", this);
  connect(button_stop_, &QPushButton::clicked, this, &StartStopPanel::stopButtonClicked);
  button_stop_->setStyleSheet("color: red; font-weight: bold;");

  // Arrange the widgets in layouts
  auto mainLayout = new QVBoxLayout();

  auto startLayout = new QHBoxLayout();
  startLayout->addWidget(label_start);
  startLayout->addWidget(topic_start_input_);
  startLayout->addWidget(button_start_);

  auto stopLayout = new QHBoxLayout();
  stopLayout->addWidget(label_stop);
  stopLayout->addWidget(topic_stop_input_);
  stopLayout->addWidget(button_stop_);

  mainLayout->addLayout(startLayout);
  mainLayout->addLayout(stopLayout);
  setLayout(mainLayout);
}

void StartStopPanel::startButtonClicked()
{
  // Publish empty message on the first topic
  std_msgs::msg::Empty empty_msg;
  for (int i = 0; i < 10; i++) {
    publisher_start_->publish(empty_msg);
  }
}

void StartStopPanel::stopButtonClicked()
{
  // Publish empty message on the second topic
  std_msgs::msg::Empty empty_msg;
  for (int i = 0; i < 10; i++) {
    publisher_stop_->publish(empty_msg);
  }
}

void StartStopPanel::updateStartTopic()
{
  // Update the topic name if it has changed
  QString new_topic = topic_start_input_->text().trimmed();
  if (!new_topic.isEmpty() && new_topic != topic_start_name_) {
    topic_start_name_ = new_topic;
    publisher_start_.reset();
    publisher_start_ = node_->create_publisher<std_msgs::msg::Empty>(
      topic_start_name_.toStdString(),
      dua_qos::Reliable::get_datum_qos());
  }
}

void StartStopPanel::updateStopTopic()
{
  // Update the topic name if it has changed
  QString new_topic = topic_stop_input_->text().trimmed();
  if (!new_topic.isEmpty() && new_topic != topic_stop_name_) {
    topic_stop_name_ = new_topic;
    publisher_stop_.reset();
    publisher_stop_ = node_->create_publisher<std_msgs::msg::Empty>(
      topic_stop_name_.toStdString(),
      dua_qos::Reliable::get_datum_qos());
  }
}

}  // namespace dua_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(dua_rviz_plugins::StartStopPanel, rviz_common::Panel)
