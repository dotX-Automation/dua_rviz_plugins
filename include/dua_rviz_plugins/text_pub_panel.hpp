#ifndef DUA_RVIZ_PLUGINS__TEXT_PUB_PANEL_HPP_
#define DUA_RVIZ_PLUGINS__TEXT_PUB_PANEL_HPP_

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>

#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

namespace dua_rviz_plugins
{

/**
 * @brief Custom panel for sending overlay text messages in RViz.
 */
class TextPubPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  /**
   * @brief Constructor for TextPubPanel.
   *
   * @param parent The parent widget.
   */
  TextPubPanel(QWidget * parent = nullptr);

  /**
   * @brief Initializes the panel when it is added to RViz.
   */
  void onInitialize() override;

private Q_SLOTS:
  /**
   * @brief Updates the topic name and creates a new publisher if necessary.
   */
  void updateTopic();

  /**
   * @brief Sends the message to the selected topic.
   */
  void sendMessage();

private:
  rclcpp::Node::SharedPtr node_; // The ROS node
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_; // The publisher

  QLabel * topic_label_; // Label for the topic input
  QLabel * status_label_; // Label for the publication status
  QLineEdit * topic_input_; // Input for the topic name
  QLineEdit * message_input_; // Input for the message
  QPushButton * send_button_; // Button to send the message

  QString topic_name_ = "/messages"; // The selected topic name
};

}  // namespace dua_rviz_plugins

#endif  // DUA_RVIZ_PLUGINS__TEXT_PUB_PANEL_HPP_
