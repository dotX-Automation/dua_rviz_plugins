/**
 * StartStopPanel class source file.
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

#include <dua_rviz_plugins/start_stop_panel.hpp>

namespace dua_rviz_plugins
{

StartStopPanel::StartStopPanel(QWidget * parent)
: rviz_common::Panel(parent),
  start_topic_("/start"),
  stop_topic_("/stop")
{
  // Initialize ROS2 node
  node_ = std::make_shared<rclcpp::Node>("rviz_button_node");

  // Create start and stop publishers
  init_start_pub(start_topic_.toStdString());
  init_stop_pub(stop_topic_.toStdString());

  // Create the UI elements for start topic
  QLabel * start_label = new QLabel("Start Topic:", this);
  start_topic_input_ = new QLineEdit(start_topic_, this);
  connect(
    start_topic_input_,
    &QLineEdit::editingFinished,
    this,
    &StartStopPanel::update_start_topic);
  QPushButton * start_button = new QPushButton("START", this);
  start_button->setStyleSheet("color: green; font-weight: bold;");
  connect(
    start_button,
    &QPushButton::clicked,
    this,
    &StartStopPanel::start_button_clicked);

  // Create the UI elements for stop topic
  QLabel * stop_label = new QLabel("Stop Topic:", this);
  stop_topic_input_ = new QLineEdit(stop_topic_, this);
  connect(
    stop_topic_input_,
    &QLineEdit::editingFinished,
    this,
    &StartStopPanel::update_stop_topic);
  QPushButton * stop_button = new QPushButton("STOP", this);
  stop_button->setStyleSheet("color: red; font-weight: bold;");
  connect(
    stop_button,
    &QPushButton::clicked,
    this,
    &StartStopPanel::stop_button_clicked);

  // Create the main layout
  auto main_layout = new QVBoxLayout();
  // Add start layout
  auto start_layout = new QHBoxLayout();
  start_layout->addWidget(start_label);
  start_layout->addWidget(start_topic_input_);
  start_layout->addWidget(start_button);
  main_layout->addLayout(start_layout);
  // Add stop layout
  auto stop_layout = new QHBoxLayout();
  stop_layout->addWidget(stop_label);
  stop_layout->addWidget(stop_topic_input_);
  stop_layout->addWidget(stop_button);
  main_layout->addLayout(stop_layout);
  // Set the main layout
  setLayout(main_layout);
}

void StartStopPanel::init_start_pub(const std::string & start_topic_)
{
  start_pub_ = node_->create_publisher<Empty>(
    start_topic_,
    dua_qos::Reliable::get_datum_qos());
}

void StartStopPanel::init_stop_pub(const std::string & stop_topic_)
{
  stop_pub_ = node_->create_publisher<Empty>(
    stop_topic_,
    dua_qos::Reliable::get_datum_qos());
}

void StartStopPanel::start_button_clicked()
{
  Empty empty_msg;
  for (int i = 0; i < REPUBLISH_COUNT; i++) {
    start_pub_->publish(empty_msg);
  }
}

void StartStopPanel::stop_button_clicked()
{
  Empty empty_msg;
  for (int i = 0; i < REPUBLISH_COUNT; i++) {
    stop_pub_->publish(empty_msg);
  }
}

void StartStopPanel::update_start_topic()
{
  // Parse the topic name from the input field
  QString new_topic_ = start_topic_input_->text().trimmed();

  // Update the topic name if it has changed
  if (!new_topic_.isEmpty() && new_topic_ != start_topic_) {
    start_topic_ = new_topic_;
    init_start_pub(start_topic_.toStdString());
  }
}

void StartStopPanel::update_stop_topic()
{
  // Parse the topic name from the input field
  QString new_topic_ = stop_topic_input_->text().trimmed();

  // Update the topic name if it has changed
  if (!new_topic_.isEmpty() && new_topic_ != stop_topic_) {
    stop_topic_ = new_topic_;
    init_stop_pub(stop_topic_.toStdString());
  }
}

}  // namespace dua_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(dua_rviz_plugins::StartStopPanel, rviz_common::Panel)
