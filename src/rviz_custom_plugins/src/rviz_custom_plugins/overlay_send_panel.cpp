#include <rviz_custom_plugins/overlay_send_panel.hpp>

namespace rviz_custom_plugins
{

/**
 * @brief Constructor for the OverlaySendPanel class.
 *
 * @param parent The parent widget.
 */
OverlaySendPanel::OverlaySendPanel(QWidget * parent)
: rviz_common::Panel(parent),
  topic_name_("/messages")
{
  // Create GUI elements
  QVBoxLayout * layout = new QVBoxLayout;

  // Create the input fields and buttons
  topic_label_ = new QLabel("Topic:");
  topic_input_ = new QLineEdit(topic_name_);
  connect(topic_input_, &QLineEdit::editingFinished, this, &OverlaySendPanel::updateTopic);

  // Create the message input field
  message_input_ = new QLineEdit();
  message_input_->setPlaceholderText("Enter message...");

  // Create the send button
  send_button_ = new QPushButton("PUBLISH");
  connect(send_button_, &QPushButton::clicked, this, &OverlaySendPanel::sendMessage);
  send_button_->setStyleSheet("color: black; font-weight: bold;");

  // Create the status label
  status_label_ = new QLabel("Ready to publish.");

  // Add the elements to the layout
  layout->addWidget(topic_label_);
  layout->addWidget(topic_input_);
  layout->addWidget(new QLabel("Message:"));
  layout->addWidget(message_input_);
  layout->addWidget(send_button_);
  layout->addWidget(status_label_);
  setLayout(layout);
}

/**
 * @brief Initializes the panel when it is added to RViz.
 */
void OverlaySendPanel::onInitialize()
{
  // Call the base class implementation
  rviz_common::Panel::onInitialize();

  // Get the ROS node associated with RViz
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
}

/**
 * @brief Updates the topic name and creates a new publisher if necessary.
 */
void OverlaySendPanel::updateTopic()
{
  // Get the new topic name
  QString new_topic = topic_input_->text().trimmed();

  // Update the topic name and reset the publisher
  if (!new_topic.isEmpty() && new_topic != topic_name_) {
    topic_name_ = new_topic;
    publisher_.reset();
  }
}

/**
 * @brief Sends the message to the selected topic.
 */
void OverlaySendPanel::sendMessage()
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
  publisher_->publish(msg);

  // Update the status label
  status_label_->setText("Message sent to " + topic_name_);
}

}  // namespace rviz_custom_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_custom_plugins::OverlaySendPanel, rviz_common::Panel)
