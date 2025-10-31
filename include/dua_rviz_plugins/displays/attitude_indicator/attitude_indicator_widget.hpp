#pragma once

#include <cmath>

#include <QWidget>
#include <QMutex>
#include <QPainter>
#include <QPaintEvent>
#include <QPainterPath>
#include <QRect>

namespace dua_rviz_plugins::displays::attitude_indicator
{

static inline double rad2deg(double r) {return r * 180.0 / M_PI;}

static inline double deg2rad(double d) {return d * M_PI / 180.0;}

class AttitudeIndicatorWidget : public QWidget
{
  Q_OBJECT

public:
  AttitudeIndicatorWidget(QWidget * parent = nullptr);
  ~AttitudeIndicatorWidget() override = default;

  void setAnchorRect(const QRect & r)
  {
    QMutexLocker lk(&mtx_);
    anchor_rect_ = r; place(); update();
  }

  void setAngles(double yaw, double pitch, double roll)
  {
    QMutexLocker lk(&mtx_);
    yaw_ = yaw; pitch_ = pitch; roll_ = roll; update();
  }

  void setScale(double s)
  {
    QMutexLocker lk(&mtx_);
    scale_ = std::clamp(s, 0.5, 3.0); place(); update();
  }

  void setMargin(int m)
  {
    QMutexLocker lk(&mtx_);
    margin_ = std::max(0, m); place(); update();
  }

  void setCorner(int idx)
  {
    QMutexLocker lk(&mtx_);
    corner_ = std::clamp(idx, 0, 3); place(); update();
  }

protected:
  void paintEvent(QPaintEvent *) override;
  bool eventFilter(QObject *, QEvent *) override;

private:
  QFont uiFont(double scale) const;
  QSize sizeHint() const override;

  void place();

  void drawCompass(QPainter & p, const QRectF & r);
  void drawHorizon(QPainter & p, const QRectF & r);
  void drawReadouts(QPainter & p, const QRect & pillRect, QFont & font);

  QMutex mtx_;
  QRect  anchor_rect_;
  double yaw_{0.0}, pitch_{0.0}, roll_{0.0};
  double scale_{1.0};
  int    margin_{8};
  int    corner_{0};
};

} // namespace dua_rviz_plugins::displays::attitude_indicator
