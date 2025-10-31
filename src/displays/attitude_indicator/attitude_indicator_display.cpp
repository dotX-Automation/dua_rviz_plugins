#include <dua_rviz_plugins/displays/attitude_indicator/attitude_indicator_display.hpp>

namespace dua_rviz_plugins::displays::attitude_indicator
{

AttitudeIndicatorDisplay::AttitudeIndicatorDisplay()
{
  prop_scale_ = new rviz_common::properties::FloatProperty(
    "Scale", 1.0, "Overlay scale factor", this);
  prop_scale_->setMin(1.0);
  prop_scale_->setMax(3.0);

  prop_margin_ = new rviz_common::properties::IntProperty(
    "Margin (px)", 10, "Distance from chosen corner", this);
  prop_margin_->setMin(0);

  prop_corner_ = new rviz_common::properties::EnumProperty(
    "Corner", "Top-Left", "Anchor corner", this);
  prop_corner_->addOption("Top-Left", 0);
  prop_corner_->addOption("Top-Right", 1);
  prop_corner_->addOption("Bottom-Left", 2);
  prop_corner_->addOption("Bottom-Right", 3);
}

AttitudeIndicatorDisplay::~AttitudeIndicatorDisplay()
{
  if (overlay_) {
    overlay_->hide();
    overlay_->deleteLater();
    overlay_ = nullptr;
  }
}

void AttitudeIndicatorDisplay::onInitialize()
{
  rviz_common::MessageFilterDisplay<geometry_msgs::msg::PoseStamped>::onInitialize();
  ensureOverlay();
}

void AttitudeIndicatorDisplay::onEnable()
{
  rviz_common::MessageFilterDisplay<geometry_msgs::msg::PoseStamped>::onEnable();
  ensureOverlay();
  if (overlay_) {overlay_->show();}
}

void AttitudeIndicatorDisplay::onDisable()
{
  if (overlay_) {overlay_->hide();}
  rviz_common::MessageFilterDisplay<geometry_msgs::msg::PoseStamped>::onDisable();
}

void AttitudeIndicatorDisplay::reset()
{
  rviz_common::MessageFilterDisplay<geometry_msgs::msg::PoseStamped>::reset();
}

void AttitudeIndicatorDisplay::update(float, float)
{
  if (!overlay_) {return;}

  if (auto * rp = context_->getViewManager()->getRenderPanel()) {
    overlay_->setAnchorRect(QRect(rp->mapToGlobal(QPoint(0, 0)), rp->size()));
  }
}

void AttitudeIndicatorDisplay::ensureOverlay()
{
  if (overlay_) {return;}

  auto * rp = context_->getViewManager()->getRenderPanel();
  if (!rp) {return;}

  // Top-level floating tool window to avoid being clipped under RViz panes
  overlay_ = new AttitudeIndicatorWidget(nullptr);
  overlay_->setWindowFlags(Qt::Tool | Qt::FramelessWindowHint | Qt::WindowStaysOnTopHint);
  overlay_->setAttribute(Qt::WA_TransparentForMouseEvents, true);
  overlay_->setAttribute(Qt::WA_NoSystemBackground, true);
  overlay_->setAutoFillBackground(false);
  overlay_->show();
  overlay_->raise();

  // Initial sync
  overlay_->setScale(prop_scale_->getFloat());
  overlay_->setMargin(prop_margin_->getInt());
  overlay_->setCorner(prop_corner_->getOptionInt());

  // Anchor to the render panel area
  overlay_->setAnchorRect(QRect(rp->mapToGlobal(QPoint(0, 0)), rp->size()));

  // Live bindings
  connect(prop_scale_, &rviz_common::properties::FloatProperty::changed, [this](){
      if(overlay_) {
        overlay_->setScale(prop_scale_->getFloat());
      }
  });
  connect(prop_margin_, &rviz_common::properties::IntProperty::changed, [this](){
      if(overlay_) {
        overlay_->setMargin(prop_margin_->getInt());
      }
  });
  connect(prop_corner_, &rviz_common::properties::EnumProperty::changed, [this](){
      if(overlay_) {
        overlay_->setCorner(prop_corner_->getOptionInt());
      }
  });
}

void AttitudeIndicatorDisplay::processMessage(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
  const auto & q = msg->pose.orientation;
  tf2::Quaternion tq(q.x, q.y, q.z, q.w);
  tf2::Matrix3x3(tq).getRPY(roll_, pitch_, yaw_);

  if (overlay_) {
    overlay_->setAngles(yaw_, pitch_, roll_);
  }
}

} // namespace dua_rviz_plugins::displays::attitude_indicator

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  dua_rviz_plugins::displays::attitude_indicator::AttitudeIndicatorDisplay,
  rviz_common::Display)
