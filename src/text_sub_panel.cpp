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
  label_ = new QLabel("Waiting for messages");
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

  // Carica la mappa dei colori da file YAML
  std::string pkg_share_dir_ = ament_index_cpp::get_package_share_directory("dua_rviz_plugins");
  std::string yaml_file_ = pkg_share_dir_ + "/config/colors.yaml";

  // Load the color configuration from a YAML file
  YAML::Node config_ = YAML::LoadFile(yaml_file_);

  for (auto it = config_.begin(); it != config_.end(); ++it) {
    std::string key_ = it->first.as<std::string>();
    std::string value_ = it->second.as<std::string>();
    color_map_[key_] = value_;
  }

  // Subscribe to the initial topic
  subscribe();
}

void TextSubPanel::subscribe()
{
  if (!topic_name_.isEmpty()) {

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
  // Set the default color
  std::string color_ = "#000000";  // Black

  // Check if the message type has a specific color
  auto it = color_map_.find(msg->data);
  if (it != color_map_.end()) {
    color_ = it->second;
  }

  // Set the text color and size
  QString text = QString("<font color='%1' style='font-size:%2px;'>%3</font>")
    .arg(color_.c_str())
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
