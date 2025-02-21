#ifndef RVIZ_CUSTOM_PLUGINS__OVERLAY_TEXT_PANEL_HPP_
#define RVIZ_CUSTOM_PLUGINS__OVERLAY_TEXT_PANEL_HPP_

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <QLabel>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>

#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

namespace rviz_custom_plugins
{

/**
 * @brief Custom panel for displaying overlay text messages in RViz.
 */
class OverlayTextPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  /**
   * @brief Constructor for OverlayTextPanel.
   *
   * @param parent The parent widget.
   */
  OverlayTextPanel(QWidget * parent = nullptr);

  /**
   * @brief Initializes the panel when it is added to RViz.
   */
  void onInitialize() override;

  /**
   * @brief Saves the panel configuration, including the selected topic.
   *
   * @param config The configuration object to store the topic name.
   */
  void save(rviz_common::Config config) const override;

  /**
   * @brief Loads the panel configuration and restores the topic subscription.
   *
   * @param config The configuration object containing the stored topic name.
   */
  void load(const rviz_common::Config & config) override;

private Q_SLOTS:
  /**
   * @brief Updates the topic subscription based on user input.
   */
  void updateTopic();

private:
  /**
   * @brief Subscribes to the selected ROS 2 topic.
   */
  void subscribe();

  /**
   * @brief Unsubscribes from the current topic to release resources.
   */
  void unsubscribe();

  /**
   * @brief Callback function that updates the UI with the received message.
   *
   * @param msg The received message containing the text data.
   */
  void callback(const std_msgs::msg::String::SharedPtr msg);

  QLabel * label_;               /**< Label to display the received message. */
  QLabel * topic_info_label_;    /**< Label to display the subscribed topic name. */
  QLineEdit * topic_input_;      /**< Input field for setting the ROS topic name. */

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_; /**< ROS 2 subscription for receiving messages. */
  rclcpp::Node::SharedPtr node_; /**< Shared pointer to the ROS 2 node used for communication. */

  QString topic_name_ = ""; /**< Stores the current topic name for subscriptions. */
};

}  // namespace rviz_custom_plugins

#endif  // RVIZ_CUSTOM_PLUGINS__OVERLAY_TEXT_PANEL_HPP_
