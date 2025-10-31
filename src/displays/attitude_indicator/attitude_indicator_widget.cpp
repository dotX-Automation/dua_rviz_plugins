#include <dua_rviz_plugins/displays/attitude_indicator/attitude_indicator_widget.hpp>

namespace dua_rviz_plugins::displays::attitude_indicator
{

AttitudeIndicatorWidget::AttitudeIndicatorWidget(QWidget * parent)
: QWidget(parent)
{
  setAttribute(Qt::WA_TransparentForMouseEvents, true);
  setAttribute(Qt::WA_TranslucentBackground, true);
  setAttribute(Qt::WA_NoSystemBackground, true);
  setAutoFillBackground(false);
  setWindowFlags(Qt::FramelessWindowHint);
}

QFont AttitudeIndicatorWidget::uiFont(double scale) const
{
  QFont f("Roboto Mono"); // one font everywhere
  f.setBold(true);
  f.setPointSizeF(8.0 * scale);
  return f;
}

QSize AttitudeIndicatorWidget::sizeHint() const
{
  const int dial_d = int(150 * scale_);
  const int dial_gap = int(5 * scale_);
  const int vpad = int(6 * scale_);
  const int hpad = int(8 * scale_);

  QFont f = uiFont(scale_);
  QFontMetrics fm(f);
  const int readout_h = int(std::round(fm.height() * 1.5));

  return QSize(2 * dial_d + dial_gap + 2 * hpad, dial_d + readout_h + 2 * vpad);
}

bool AttitudeIndicatorWidget::eventFilter(QObject *, QEvent * ev)
{
  if (ev->type() == QEvent::Resize || ev->type() == QEvent::Show) {place();}
  return false;
}

void AttitudeIndicatorWidget::place()
{
  const QSize sz = sizeHint();
  const int w = sz.width();
  const int h = sz.height();

  if (anchor_rect_.isValid()) {
    const int x0 = anchor_rect_.x();
    const int y0 = anchor_rect_.y();
    const int W = anchor_rect_.width();
    const int H = anchor_rect_.height();

    int gx = x0 + margin_;
    int gy = y0 + margin_;
    if      (corner_ == 1) {gx = x0 + W - w - margin_; gy = y0 + margin_;} else if (corner_ == 2) {
      gx = x0 + margin_;         gy = y0 + H - h - margin_;
    } else if (corner_ == 3) {gx = x0 + W - w - margin_; gy = y0 + H - h - margin_;}

    move(gx, gy);
    resize(w, h);
  }
}

void AttitudeIndicatorWidget::paintEvent(QPaintEvent *)
{
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing, true);
  p.setCompositionMode(QPainter::CompositionMode_SourceOver);

  // Layout
  QFont baseFont = uiFont(scale_);
  p.setFont(baseFont);
  QFontMetrics fm(baseFont);

  const int vpad = int(6 * scale_);
  const int hpad = int(8 * scale_);
  const int dial_gap = int(6 * scale_);
  const int readout_h = int(std::round(fm.height() * 1.5));

  QRect content = rect().adjusted(hpad, vpad, -hpad, -(readout_h + vpad));
  const int dial_d = std::min(content.height(), (content.width() - dial_gap) / 2);
  const int dial_y = content.y() + (content.height() - dial_d) / 2;
  const int left_x = content.x();
  const int right_x = content.x() + dial_d + dial_gap;

  QRectF left (left_x, dial_y, dial_d, dial_d);
  QRectF right(right_x, dial_y, dial_d, dial_d);

  // COMPASS (clipped content)
  {
    QPainterPath clip; clip.addEllipse(left);
    p.setClipPath(clip);
    drawCompass(p, left);
    p.setClipping(false);
  }

  // HORIZON (clipped content)
  {
    QPainterPath clip; clip.addEllipse(right);
    p.setClipPath(clip);
    drawHorizon(p, right);
    p.setClipping(false);
  }

  // Readout pill (kept as-is)
  const QString readout_template = "ROLL  000.0°    PITCH  000.0°    YAW  000.0°";
  int pill_w = std::min(rect().width() - 2 * hpad,
                        int(fm.horizontalAdvance(readout_template) * 1.15));
  QRect pill(rect().center().x() - pill_w / 2,
    rect().bottom() - readout_h - vpad / 2,
    pill_w, readout_h);
  drawReadouts(p, pill, baseFont);
}

void AttitudeIndicatorWidget::drawCompass(QPainter & p, const QRectF & r)
{
  QMutexLocker lk(&mtx_);
  QFontMetrics fm(p.font());
  const QPointF c = r.center();
  const double R = std::min(r.width(), r.height()) * 0.48;
  p.setPen(Qt::NoPen);
  p.setBrush(QColor(20, 20, 20, 200));
  p.drawEllipse(c, R, R);
  for (int a = 0; a < 360; a += 5) {
    const double rad = deg2rad(a);
    const double sa = std::sin(rad), ca = std::cos(rad);
    QPointF p1, p2(c.x() + R * sa, c.y() - R * ca);
    double r = 0.0;
    if (a % 30 == 0) {
      if (a % 90 == 0) {
        r = 0.3 * R;
        const QString text = (a == 0) ? "N" : (a == 90) ? "E" : (a == 180) ? "S" : "W";
        const int tw = fm.horizontalAdvance(text);
        const int th = fm.height();
        const QPointF pos(c.x() + r * sa - tw / 2, c.y() - r * ca - th / 2);
        p.setPen(QPen(QColor(255, 255, 255), 2.0 * scale_));
        p.drawText(QRectF(pos.x(), pos.y(), tw, th), Qt::AlignCenter, text);
      }
      r = 0.65 * R;
      const QString text = QString::number(-rad2deg(std::atan2(sa, ca)));
      const int tw = fm.horizontalAdvance(text);
      const int th = fm.height();
      const QPointF pos(c.x() + r * sa - tw / 2, c.y() - r * ca - th / 2);
      const QRectF bb(pos.x(), pos.y(), tw, th);
      p.setPen(QPen(QColor(200, 200, 200), 1.0 * scale_));
      p.drawText(bb, Qt::AlignCenter, text);
      r = 0.85 * R;
      p1 = QPointF(c.x() + r * sa, c.y() - r * ca);
      p.setPen(QPen(QColor(255, 255, 255), 2.0 * scale_));
    } else {
      r = 0.9 * R;
      p1 = QPointF(c.x() + r * sa, c.y() - r * ca);
      p.setPen(QPen(QColor(200, 200, 200), 1.0 * scale_));
    }
    p.drawLine(p1, p2);
  }
  p.setPen(QPen(QColor(255, 255, 255), 3.0 * scale_));
  p.setBrush(Qt::NoBrush);
  p.drawEllipse(c, R, R);
  const double w = 8.0 * scale_;
  {
    p.save();
    p.translate(c);
    p.rotate(-rad2deg(yaw_));
    p.setPen(QPen(QColor(255, 255, 255), 1.0 * scale_));
    QPolygonF red_tri;
    red_tri << QPointF(0, -R)
            << QPointF(0 - w, 0)
            << QPointF(0 + w, 0);
    p.setBrush(QColor(255, 0, 0, 255));
    p.drawPolygon(red_tri);
    QPolygonF blue_tri;
    blue_tri << QPointF(0, R)
             << QPointF(0 - w, 0)
             << QPointF(0 + w, 0);
    p.setBrush(QColor(0, 0, 255, 255));
    p.drawPolygon(blue_tri);
    p.restore();
  }
  p.setBrush(QColor(0, 0, 0, 255));
  p.setPen(QPen(QColor(255, 255, 255), 1.0 * scale_));
  p.drawEllipse(c, w / 2, w / 2);
}

void AttitudeIndicatorWidget::drawHorizon(QPainter & p, const QRectF & r)
{
  QMutexLocker lk(&mtx_);
  QFontMetrics fm(p.font());
  const QPointF c = r.center();
  const double R = std::min(r.width(), r.height()) * 0.48;
  const double rr = 0.75 * R;
  const double ppd = (2 * R) / 60.0;

  {
    p.save();
    p.translate(c);
    p.rotate(-rad2deg(roll_));
    {
      p.save();
      {
        QPainterPath clip;
        clip.addEllipse(QPointF(0, 0), R, R);
        p.setClipPath(clip);
        p.translate(0, rad2deg(pitch_) * ppd);
        QLinearGradient skyGrad(QPointF(0, -R), QPointF(0, 0));
        skyGrad.setColorAt(0, QColor(0, 127, 255));
        skyGrad.setColorAt(1, QColor(0, 90, 200));
        QLinearGradient groundGrad(QPointF(0, 0), QPointF(0, R));
        groundGrad.setColorAt(0, QColor(40, 140, 40));
        groundGrad.setColorAt(1, QColor(20, 100, 20));
        const double W = 2 * R;
        const double X = -R;
        p.fillRect(QRectF(X, -3 * R, W, 3 * R), skyGrad);
        p.fillRect(QRectF(X, 0, W, 3 * R), groundGrad);

        p.setPen(QPen(Qt::white, 2.0 * scale_));
        p.drawLine(QPointF(-R, 0), QPointF(R, 0));
        p.setClipping(false);
      }
      {
        QPainterPath clip;
        clip.addEllipse(QPointF(0, -rad2deg(pitch_) * ppd), rr, rr);
        p.setClipPath(clip);
        for (int d = -60; d <= 60; d += 5) {
          const double y = -d * ppd;
          const bool major = (d % 10) == 0;
          const double w = major ? 0.3 * R : 0.2 * R;
          p.drawLine(QPointF(-w, y), QPointF(w, y));
          if (major && d != 0) {
            const QString text = QString::number(std::abs(d));
            const int tw = fm.horizontalAdvance(text);
            const int th = fm.height();
            p.drawText(QRectF(-w - tw - 2 * scale_, y - th / 2, tw, th),
                Qt::AlignCenter, text);
            p.drawText(QRectF(w + 2 * scale_, y - th / 2, tw, th),
                Qt::AlignCenter, text);
          }
        }
        p.setClipping(false);
      }
      p.restore();
    }
    p.setPen(QPen(QColor(255, 255, 255), 1.5 * scale_));
    std::vector<double> a = {0, 30, 60, 70, 80, 90, 100, 110, 120, 150, 180};
    std::vector<double> w = {2, 2, 2, 1, 1, 2, 1, 1, 2, 2, 2};
    std::vector<double> l = {0.95, 0.95, 0.95, 0.85, 0.85, 0.95, 0.85, 0.85, 0.95, 0.95, 0.95};
    for (size_t i = 0; i < a.size(); ++i) {
      const double rad = deg2rad(a[i] - 90);
      const double sa = std::sin(rad), ca = std::cos(rad);
      const QPointF p1(rr * sa, -rr * ca);
      const QPointF p2(l[i] * R * sa, -l[i] * R * ca);
      p.setPen(QPen(QColor(255, 255, 255), w[i] * scale_));
      p.drawLine(p1, p2);
    }
    p.restore();
  }

  p.setPen(QPen(QColor(255, 255, 0, 255), 3.0 * scale_));
  p.setBrush(QColor(255, 255, 0, 255));
  p.drawLine(QPointF(c.x() - R * 0.3, c.y()), QPointF(c.x(), c.y()));
  p.drawLine(QPointF(c.x() + R * 0.3, c.y()), QPointF(c.x(), c.y()));
  p.drawLine(QPointF(c.x() - R * 0.3, c.y() - R * 0.05),
      QPointF(c.x() - R * 0.3, c.y() + R * 0.05));
  p.drawLine(QPointF(c.x() + R * 0.3, c.y() - R * 0.05),
      QPointF(c.x() + R * 0.3, c.y() + R * 0.05));
  p.drawEllipse(c, 4.0 * scale_, 4.0 * scale_);
  QPolygonF tri;
  tri << QPointF(c.x(), c.y() - R)
      << QPointF(c.x() + 0.075 * R, c.y() - 0.75 * R)
      << QPointF(c.x() - 0.075 * R, c.y() - 0.75 * R);
  p.drawPolygon(tri);

  p.setBrush(Qt::NoBrush);
  p.setPen(QPen(QColor(255, 255, 255), 3.0 * scale_));
  p.drawEllipse(c, R, R);
}

/* =================== READOUT PILL =================== */

void AttitudeIndicatorWidget::drawReadouts(QPainter & p, const QRect & pillRect, QFont & font)
{
  QMutexLocker lk(&mtx_);

  const int radius = std::min(pillRect.height() / 2, int(12 * scale_));
  p.setBrush(QColor(0, 0, 0, 125));
  p.setPen(Qt::NoPen);
  p.drawRoundedRect(pillRect, radius, radius);

  const QString text = QString("ROLL  %1°   PITCH  %2°   YAW  %3°")
    .arg(QString::number(rad2deg(roll_), 'f', 1))
    .arg(QString::number(rad2deg(pitch_), 'f', 1))
    .arg(QString::number(rad2deg(yaw_), 'f', 1));
  p.setPen(Qt::white);
  p.setFont(font);
  p.drawText(pillRect, Qt::AlignHCenter | Qt::AlignVCenter, text);
}

} // namespace dua_rviz_plugins::displays::attitude_indicator
