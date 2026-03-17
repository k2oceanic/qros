#pragma once
#include "qros_defs.h"

#include <QObject>
#include <QVector3D>
#include <QQuaternion>
#include <QMatrix4x4>
#include <QDateTime>

#include <geometry_msgs/msg/transform_stamped.hpp>

QROS_NS_HEAD

    class QRosTransformStamped : public QObject {
  Q_OBJECT
  Q_PROPERTY(bool        valid        READ valid        WRITE setValid        NOTIFY validChanged)
  Q_PROPERTY(QString     frameId      READ frameId      WRITE setFrameId      NOTIFY frameIdChanged)
  Q_PROPERTY(QString     childFrameId READ childFrameId WRITE setChildFrameId NOTIFY childFrameIdChanged)
  Q_PROPERTY(double      stampSec     READ stampSec     WRITE setStampSec     NOTIFY stampSecChanged)  // seconds (double)
  Q_PROPERTY(QDateTime   stamp        READ stamp        WRITE setStamp        NOTIFY stampChanged)      // Qt UTC time
  Q_PROPERTY(QVector3D   translation  READ translation  WRITE setTranslation  NOTIFY translationChanged)
  Q_PROPERTY(QQuaternion rotation     READ rotation     WRITE setRotation     NOTIFY rotationChanged)
  Q_PROPERTY(QMatrix4x4  matrix       READ matrix                              NOTIFY matrixChanged)

public:
  explicit QRosTransformStamped(QObject* parent=nullptr) : QObject(parent) {
    m_.setToIdentity();
  }

  // Factory helpers
  static QRosTransformStamped* fromMsg(const geometry_msgs::msg::TransformStamped& tf, QObject* parent=nullptr) {
    auto *obj = new QRosTransformStamped(parent);
    obj->valid_ = true;
    obj->frame_id_ = QString::fromStdString(tf.header.frame_id);
    obj->child_frame_id_ = QString::fromStdString(tf.child_frame_id);
    const double sec = static_cast<double>(tf.header.stamp.sec) +
                       static_cast<double>(tf.header.stamp.nanosec) * 1e-9;
    obj->setStampSec(sec); // keeps QDateTime in sync
    obj->t_ = QVector3D(tf.transform.translation.x,
                        tf.transform.translation.y,
                        tf.transform.translation.z);
    obj->q_ = QQuaternion(tf.transform.rotation.w,
                          tf.transform.rotation.x,
                          tf.transform.rotation.y,
                          tf.transform.rotation.z);
    obj->updateMatrix_();
    return obj;
  }

  static QRosTransformStamped* invalid(QObject* parent=nullptr) {
    auto *obj = new QRosTransformStamped(parent);
    obj->valid_ = false;
    obj->stamp_ = QDateTime(); // invalid
    return obj;
  }

  // Getters
  bool        valid()        const { return valid_; }
  QString     frameId()      const { return frame_id_; }
  QString     childFrameId() const { return child_frame_id_; }
  double      stampSec()     const { return stamp_sec_; }
  QDateTime   stamp()        const { return stamp_; }
  QVector3D   translation()  const { return t_; }
  QQuaternion rotation()     const { return q_; }
  QMatrix4x4  matrix()       const { return m_; }

  // Setters (QML-writable)
  void setValid(bool v) {
    if (valid_ == v) return;
    valid_ = v;
    emit validChanged();
  }

  void setFrameId(const QString& s) {
    if (frame_id_ == s) return;
    frame_id_ = s;
    emit frameIdChanged();
  }

  void setChildFrameId(const QString& s) {
    if (child_frame_id_ == s) return;
    child_frame_id_ = s;
    emit childFrameIdChanged();
  }

  void setStampSec(double s) {
    if (qFuzzyCompare(stamp_sec_, s)) return;
    stamp_sec_ = s;
    // Keep QDateTime in sync (UTC)
    stamp_ = QDateTime::fromMSecsSinceEpoch(static_cast<qint64>(stamp_sec_ * 1000.0), Qt::UTC);
    emit stampSecChanged();
    emit stampChanged();
  }

  void setStamp(const QDateTime& dt) {
    if (stamp_ == dt) return;
    stamp_ = dt;
    // Keep seconds in sync
    stamp_sec_ = static_cast<double>(dt.toMSecsSinceEpoch()) / 1000.0;
    emit stampChanged();
    emit stampSecChanged();
  }

  void setTranslation(const QVector3D& v) {
    if (t_ == v) return;
    t_ = v;
    updateMatrix_();
    emit translationChanged();
  }

  void setRotation(const QQuaternion& q) {
    if (q_ == q) return;
    q_ = q;
    updateMatrix_();
    emit rotationChanged();
  }

signals:
  void validChanged();
  void frameIdChanged();
  void childFrameIdChanged();
  void stampSecChanged();
  void stampChanged();
  void translationChanged();
  void rotationChanged();
  void matrixChanged();

private:
  void updateMatrix_() {
    m_.setToIdentity();
    m_.translate(t_);
    m_.rotate(q_);
    emit matrixChanged();
  }

  bool        valid_ = false;
  QString     frame_id_;
  QString     child_frame_id_;
  double      stamp_sec_ = 0.0;
  QDateTime   stamp_;
  QVector3D   t_;
  QQuaternion q_;
  QMatrix4x4  m_;
};

QROS_NS_FOOT
