#include <rviz_custom_plugins/overlay_text_panel.hpp>

namespace rviz_custom_plugins
{

/**
 * @brief Constructor for the OverlayTextPanel class.
 *
 * @param parent The parent widget.
 */
OverlayTextPanel::OverlayTextPanel(QWidget * parent)
: rviz_common::Panel(parent),
  topic_name_("/messages")
{
  // Create GUI elements
  QVBoxLayout * layout = new QVBoxLayout;

  // Connect signals and slots
  topic_input_ = new QLineEdit("/messages");
  connect(topic_input_, &QLineEdit::editingFinished, this, &OverlayTextPanel::updateTopic);

  label_ = new QLabel("Waiting for message...");
  layout->addWidget(new QLabel("Topic:"));
  layout->addWidget(topic_input_);
  layout->addWidget(label_);
  setLayout(layout);
}

/**
 * @brief Initializes the panel when it is added to RViz.
 */
void OverlayTextPanel::onInitialize()
{
  rviz_common::Panel::onInitialize();

  // Get the ROS node associated with RViz
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  // Subscribe to the initial topic
  subscribe();
}

/**
 * @brief Subscribes to the current topic and updates the UI.
 */
void OverlayTextPanel::subscribe()
{
  if (!topic_name_.isEmpty()) {
    RCLCPP_INFO(node_->get_logger(), "Subscribing to: %s", topic_name_.toStdString().c_str());

    // Create a new subscription
    subscription_ = node_->create_subscription<std_msgs::msg::String>(
      topic_name_.toStdString(), 10,
      std::bind(&OverlayTextPanel::callback, this, std::placeholders::_1));

    label_->setText("Subscribed to: " + topic_name_);
  }
}

/**
 * @brief Unsubscribes from the current topic to release resources.
 */
void OverlayTextPanel::unsubscribe()
{
  // Reset the subscription
  subscription_.reset();
}

/**
 * @brief Updates the topic subscription based on the user input.
 */
void OverlayTextPanel::updateTopic()
{
  // Get the new topic name from the input field
  QString new_topic = topic_input_->text().trimmed();

  if (!new_topic.isEmpty() && new_topic != topic_name_) {
    // Unsubscribe from the current topic
    unsubscribe();
    // Update the topic name
    topic_name_ = new_topic;

    // Subscribe to the new topic
    subscribe();
  }
}

/**
 * @brief Callback function that updates the UI with the received message.
 *
 * @param msg The received message containing the text data.
 */
void OverlayTextPanel::callback(const std_msgs::msg::String::SharedPtr msg)
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

  std::string color = "#000000";  // Default: black
  auto it = color_map.find(msg->data);
  if (it != color_map.end()) {
    color = it->second;
  }

  // Set the text color and size
  QString text = QString("<font color='%1' style='font-size:%2px;'>%3</font>")
    .arg(color.c_str())
    .arg(20)
    .arg(msg->data.c_str());

  label_->setText(text);
}

/**
 * @brief Saves the panel configuration, including the selected topic.
 *
 * @param config The configuration object to store the topic name.
 */
void OverlayTextPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("Topic", topic_name_);
}

/**
 * @brief Loads the panel configuration and restores the topic subscription.
 *
 * @param config The configuration object containing the stored topic name.
 */
void OverlayTextPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  QString topic;
  if (config.mapGetString("Topic", &topic)) {
    topic_input_->setText(topic);
    updateTopic();
  }
}

}  // namespace rviz_custom_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_custom_plugins::OverlayTextPanel, rviz_common::Panel)
