#include <dua_rviz_plugins/text_sub_panel.hpp>

namespace dua_rviz_plugins
{

TextSubPanel::TextSubPanel(QWidget * parent)
: rviz_common::Panel(parent),
  topic_name_("/messages")
{
  // Create GUI elements
  QVBoxLayout * layout = new QVBoxLayout;

  // Connect signals and slots
  topic_input_ = new QLineEdit("/messages");
  connect(topic_input_, &QLineEdit::editingFinished, this, &TextSubPanel::updateTopic);

  // Create a label to display the message
  label_ = new QLabel("Waiting for message...");
  layout->addWidget(new QLabel("Topic:"));
  layout->addWidget(topic_input_);
  layout->addWidget(label_);
  setLayout(layout);
}

void TextSubPanel::onInitialize()
{
  // Call the base class implementation
  rviz_common::Panel::onInitialize();

  // Get the ROS node associated with RViz
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  // Subscribe to the initial topic
  subscribe();
}

void TextSubPanel::subscribe()
{
  if (!topic_name_.isEmpty()) {
    RCLCPP_INFO(node_->get_logger(), "Subscribing to: %s", topic_name_.toStdString().c_str());

    // Create a new subscription
    subscription_ = node_->create_subscription<std_msgs::msg::String>(
      topic_name_.toStdString(), 10,
      std::bind(&TextSubPanel::callback, this, std::placeholders::_1));

    label_->setText("Subscribed to: " + topic_name_);
  }
}

void TextSubPanel::unsubscribe()
{
  // Reset the subscription
  subscription_.reset();
}

void TextSubPanel::updateTopic()
{
  // Get the new topic name from the input field
  QString new_topic = topic_input_->text().trimmed();

  // Check if the topic name has changed
  if (!new_topic.isEmpty() && new_topic != topic_name_) {
    // Unsubscribe from the current topic
    unsubscribe();
    // Update the topic name
    topic_name_ = new_topic;

    // Subscribe to the new topic
    subscribe();
  }
}

void TextSubPanel::callback(const std_msgs::msg::String::SharedPtr msg)
{
  // Define the color for each message type
  static const std::map<std::string, std::string> color_map = {
    {"EMERGENCY_LANDING", "#8b0000"},  // Dark red
    {"RTB", "#8b0000"},                // Dark red
    {"TAKEOFF", "#feb000"},            // Dark yellow
    {"ARM", "#feb000"},                // Dark yellow
    {"DISARM", "#feb000"},             // Dark yellow
    {"EXPLORE", "#00008b"},            // Dark blue
    {"TRACK", "#008b8b"},              // Dark cyan
    {"FOLLOWME", "#cc8400"},           // Dark orange
    {"COMPLETED", "#006400"}           // Dark green
  };

  // Set the default color
  std::string color = "#000000";  // Black
  auto it = color_map.find(msg->data);
  if (it != color_map.end()) {
    color = it->second;
  }

  // Set the text color and size
  QString text = QString("<font color='%1' style='font-size:%2px;'>%3</font>")
    .arg(color.c_str())
    .arg(20)
    .arg(msg->data.c_str());

  // Update the label with the received message
  label_->setText(text);
}

void TextSubPanel::save(rviz_common::Config config) const
{
  // Call the base class implementation
  rviz_common::Panel::save(config);

  // Store the selected topic name
  config.mapSetValue("Topic", topic_name_);
}

void TextSubPanel::load(const rviz_common::Config & config)
{
  // Call the base class implementation
  rviz_common::Panel::load(config);

  // Load the stored topic name
  QString topic;
  if (config.mapGetString("Topic", &topic)) {
    topic_input_->setText(topic);
    updateTopic();
  }
}

}  // namespace dua_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(dua_rviz_plugins::TextSubPanel, rviz_common::Panel)
