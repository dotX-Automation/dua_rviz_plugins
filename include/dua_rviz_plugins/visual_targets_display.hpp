#ifndef DUA_RVIZ_PLUGINS__VISUAL_TARGETS_DISPLAY_HPP_
#define DUA_RVIZ_PLUGINS__VISUAL_TARGETS_DISPLAY_HPP_

#include <dua_qos_cpp/dua_qos.hpp>
#include <dua_interfaces/msg/visual_targets.hpp>

#include <rviz_common/ros_topic_display.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <tuple>

#include <QDialog>
#include <QImage>
#include <QLabel>
#include <QObject>

#include <interactive_markers/interactive_marker_server.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>

namespace dua_rviz_plugins
{

using geometry_msgs::msg::Pose;
using sensor_msgs::msg::Image;

typedef std::tuple<std::string, Pose, Image> Info;
typedef std::vector<Info> Infos;

/**
 * @brief Display visual targets in RViz.
 */
class VisualTargetsDisplay
  : public rviz_common::RosTopicDisplay<dua_interfaces::msg::VisualTargets>
{
  Q_OBJECT

public:
  /**
   * @brief Constructor.
   */
  VisualTargetsDisplay();
  /**
   * @brief Destructor.
   */
  ~VisualTargetsDisplay() override;

protected:
  /**
   * @brief Initialize the display.
   */
  void onInitialize() override;
  /**
   * @brief Process the received message.
   */
  void processMessage(dua_interfaces::msg::VisualTargets::ConstSharedPtr msg) override;

private:
  /**
   * @brief Create an interactive marker.
   */
  void createInteractiveMarker(
    const geometry_msgs::msg::Pose & pose,
    const std::string & id);
  /**
   * @brief Process the interactive marker feedback.
   */
  void processFeedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback,
    const std::string & id);
  /**
   * @brief Show the image in a dialog.
   */
  void showImage(const std::string & id);

  std::string frame_id_; //*< The frame ID of the visual targets. >*/
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_; //*< The interactive marker server. >*/
  std::unordered_map<std::string, Infos> map_; /**< The frame ID of map. >*/
  std::mutex mutex_; //*< The mutex to protect the map. >*/
};

}  // namespace dua_rviz_plugins

#endif  // DUA_RVIZ_PLUGINS__VISUAL_TARGETS_DISPLAY_HPP_
