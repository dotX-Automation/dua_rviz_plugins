#include <dua_rviz_plugins/displays/attitude_indicator/attitude_indicator_display.hpp>

namespace dua_rviz_plugins::displays::attitude_indicator
{

AttitudeIndicatorDisplay::AttitudeIndicatorDisplay()
{
  prop_msg_type_ = new rviz_common::properties::EnumProperty(
    "Message Type", "PoseStamped",
    "Type of incoming orientation message", this,
    SLOT(onMsgTypeChanged()), this);
  prop_msg_type_->addOption("Imu",         0);
  prop_msg_type_->addOption("Odometry",    1);
  prop_msg_type_->addOption("PoseStamped", 2);

  prop_topic_ = new rviz_common::properties::RosTopicProperty(
    "Topic", "",
    QString::fromStdString(
      rosidl_generator_traits::name<geometry_msgs::msg::PoseStamped>()),
    "Topic to subscribe to", this,
    SLOT(onTopicChanged()), this);

  prop_scale_ = new rviz_common::properties::FloatProperty(
    "Scale", 1.0, "Overlay scale factor", this);
  prop_scale_->setMin(1.0);
  prop_scale_->setMax(3.0);

  prop_margin_ = new rviz_common::properties::IntProperty(
    "Margin", 10, "Distance from chosen corner", this);
  prop_margin_->setMin(0);

  prop_corner_ = new rviz_common::properties::EnumProperty(
    "Corner", "Top-Left", "Anchor corner", this);
  prop_corner_->addOption("Top-Left",     0);
  prop_corner_->addOption("Top-Right",    1);
  prop_corner_->addOption("Bottom-Left",  2);
  prop_corner_->addOption("Bottom-Right", 3);
}

AttitudeIndicatorDisplay::~AttitudeIndicatorDisplay()
{
  unsubscribeAll();
  if (overlay_) {
    overlay_->hide();
    overlay_->deleteLater();
    overlay_ = nullptr;
  }
}

void AttitudeIndicatorDisplay::onInitialize()
{
  rviz_common::Display::onInitialize();
  prop_topic_->initialize(context_->getRosNodeAbstraction());
  ensureOverlay();
}

void AttitudeIndicatorDisplay::onEnable()
{
  ensureOverlay();
  resubscribe();
  if (overlay_) {
    std::lock_guard<std::mutex> lk(angles_mutex_);
    overlay_->setAngles(yaw_, pitch_, roll_);
    overlay_->show();
  }
}

void AttitudeIndicatorDisplay::onDisable()
{
  unsubscribeAll();
  {
    std::lock_guard<std::mutex> lk(angles_mutex_);
    roll_ = pitch_ = yaw_ = 0.0;
  }
  if (overlay_) {
    overlay_->hide();
    overlay_->setAngles(0.0, 0.0, 0.0);
  }
}

void AttitudeIndicatorDisplay::reset()
{
  rviz_common::Display::reset();
  {
    std::lock_guard<std::mutex> lk(angles_mutex_);
    roll_ = pitch_ = yaw_ = 0.0;
  }
  if (overlay_) {
    overlay_->setAngles(0.0, 0.0, 0.0);
  }
}

void AttitudeIndicatorDisplay::update(float, float)
{
  if (!overlay_) {return;}

  if (auto * rp = context_->getViewManager()->getRenderPanel()) {
    overlay_->setAnchorRect(QRect(rp->mapToGlobal(QPoint(0, 0)), rp->size()));
  }

  double r, p, y;
  {
    std::lock_guard<std::mutex> lk(angles_mutex_);
    r = roll_; p = pitch_; y = yaw_;
  }
  overlay_->setAngles(y, p, r);
}

void AttitudeIndicatorDisplay::ensureOverlay()
{
  if (overlay_) {return;}

  auto * rp = context_->getViewManager()->getRenderPanel();
  if (!rp) {return;}

  overlay_ = new AttitudeIndicatorWidget(nullptr);
  overlay_->setWindowFlags(Qt::Tool | Qt::FramelessWindowHint);
  overlay_->setAttribute(Qt::WA_TransparentForMouseEvents, true);
  overlay_->setAttribute(Qt::WA_NoSystemBackground, true);
  overlay_->setAutoFillBackground(false);
  overlay_->show();
  overlay_->raise();

  overlay_->setScale(prop_scale_->getFloat());
  overlay_->setMargin(prop_margin_->getInt());
  overlay_->setCorner(prop_corner_->getOptionInt());

  overlay_->setAnchorRect(QRect(rp->mapToGlobal(QPoint(0, 0)), rp->size()));

  connect(prop_scale_, &rviz_common::properties::FloatProperty::changed, [this](){
      if (overlay_) { overlay_->setScale(prop_scale_->getFloat()); }
    });
  connect(prop_margin_, &rviz_common::properties::IntProperty::changed, [this](){
      if (overlay_) { overlay_->setMargin(prop_margin_->getInt()); }
    });
  connect(prop_corner_, &rviz_common::properties::EnumProperty::changed, [this](){
      if (overlay_) { overlay_->setCorner(prop_corner_->getOptionInt()); }
    });
}

void AttitudeIndicatorDisplay::unsubscribeAll()
{
  sub_imu_.reset();
  sub_odom_.reset();
  sub_pose_.reset();
}

void AttitudeIndicatorDisplay::resubscribe()
{
  unsubscribeAll();

  if (!isEnabled()) {return;}

  const std::string topic = prop_topic_->getTopicStd();
  if (topic.empty()) {
    setStatus(
      rviz_common::properties::StatusProperty::Warn, "Topic", "No topic set");
    return;
  }

  auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();
  const rclcpp::QoS qos(10);

  try {
    switch (prop_msg_type_->getOptionInt()) {
      case 0:
        sub_imu_ = node->create_subscription<sensor_msgs::msg::Imu>(
          topic, qos,
          [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) { cbImu(msg); });
        break;
      case 1:
        sub_odom_ = node->create_subscription<nav_msgs::msg::Odometry>(
          topic, qos,
          [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { cbOdometry(msg); });
        break;
      default:
        sub_pose_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
          topic, qos,
          [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) { cbPoseStamped(msg); });
        break;
    }
    setStatus(rviz_common::properties::StatusProperty::Ok, "Topic", "Subscribed");
  } catch (const rclcpp::exceptions::InvalidTopicNameError & e) {
    setStatus(
      rviz_common::properties::StatusProperty::Error, "Topic",
      QString("Invalid topic: ") + e.what());
  }
}

void AttitudeIndicatorDisplay::onMsgTypeChanged()
{
  QString type_str;
  switch (prop_msg_type_->getOptionInt()) {
    case 0:
      type_str = QString::fromStdString(
        rosidl_generator_traits::name<sensor_msgs::msg::Imu>());
      break;
    case 1:
      type_str = QString::fromStdString(
        rosidl_generator_traits::name<nav_msgs::msg::Odometry>());
      break;
    default:
      type_str = QString::fromStdString(
        rosidl_generator_traits::name<geometry_msgs::msg::PoseStamped>());
      break;
  }
  prop_topic_->setMessageType(type_str);
  resubscribe();
}

void AttitudeIndicatorDisplay::onTopicChanged()
{
  resubscribe();
}

void AttitudeIndicatorDisplay::processQuaternion(double x, double y, double z, double w)
{
  tf2::Quaternion tq(x, y, z, w);
  const bool finite =
    std::isfinite(tq.x()) && std::isfinite(tq.y()) &&
    std::isfinite(tq.z()) && std::isfinite(tq.w());
  const double len2 = tq.length2();

  double r = 0.0, p = 0.0, y_val = 0.0;
  if (finite && len2 > 1e-12) {
    tq.normalize();
    tf2::Matrix3x3(tq).getRPY(r, p, y_val);
  }

  std::lock_guard<std::mutex> lk(angles_mutex_);
  roll_ = r; pitch_ = p; yaw_ = y_val;
}

void AttitudeIndicatorDisplay::cbImu(sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  const auto & q = msg->orientation;
  processQuaternion(q.x, q.y, q.z, q.w);
}

void AttitudeIndicatorDisplay::cbOdometry(nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  const auto & q = msg->pose.pose.orientation;
  processQuaternion(q.x, q.y, q.z, q.w);
}

void AttitudeIndicatorDisplay::cbPoseStamped(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
  const auto & q = msg->pose.orientation;
  processQuaternion(q.x, q.y, q.z, q.w);
}

} // namespace dua_rviz_plugins::displays::attitude_indicator

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  dua_rviz_plugins::displays::attitude_indicator::AttitudeIndicatorDisplay,
  rviz_common::Display)
