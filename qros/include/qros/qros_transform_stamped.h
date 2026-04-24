#pragma once

/**
 * @file qros_transform_stamped.h
 * @brief QML-exposed wrapper for geometry_msgs/msg/TransformStamped.
 */

#include "qros_defs.h"

#include <QObject>
#include <QVector3D>
#include <QQuaternion>
#include <QMatrix4x4>
#include <QDateTime>

#include <geometry_msgs/msg/transform_stamped.hpp>

QROS_NS_HEAD

/**
 * @brief Immutable value object representing a single stamped coordinate transform.
 *
 * Created by QRosTfBuffer::lookupTransform() and owned by QML.  Exposes
 * translation, rotation, and a pre-computed 4×4 transformation matrix
 * for direct use in Qt 3D or shader-based visualisation.
 *
 * The `valid` property is false when the lookup failed; all other fields
 * are undefined in that case.
 *
 * ### QML usage
 * @code{.qml}
 * property var tf: tfBuffer.lookupTransform("world", "tool")
 * Text { text: tf.valid ? tf.translation.toString() : "no transform" }
 * @endcode
 */
class QRosTransformStamped : public QObject {
  Q_OBJECT
  /// False if the lookup that produced this object failed.
  Q_PROPERTY(bool        valid        READ valid        WRITE setValid        NOTIFY validChanged)
  /// Header frame ID of the parent (target) frame.
  Q_PROPERTY(QString     frameId      READ frameId      WRITE setFrameId      NOTIFY frameIdChanged)
  /// Child (source) frame ID.
  Q_PROPERTY(QString     childFrameId READ childFrameId WRITE setChildFrameId NOTIFY childFrameIdChanged)
  /// Timestamp as a double (seconds since epoch).
  Q_PROPERTY(double      stampSec     READ stampSec     WRITE setStampSec     NOTIFY stampSecChanged)
  /// Timestamp as a QDateTime (UTC).
  Q_PROPERTY(QDateTime   stamp        READ stamp        WRITE setStamp        NOTIFY stampChanged)
  /// Translation vector (x, y, z) in metres.
  Q_PROPERTY(QVector3D   translation  READ translation  WRITE setTranslation  NOTIFY translationChanged)
  /// Rotation quaternion (w, x, y, z).
  Q_PROPERTY(QQuaternion rotation     READ rotation     WRITE setRotation     NOTIFY rotationChanged)
  /// Pre-computed 4×4 homogeneous transform matrix (read-only).
  Q_PROPERTY(QMatrix4x4  matrix       READ matrix                              NOTIFY matrixChanged)

public:
  explicit QRosTransformStamped(QObject* parent=nullptr) : QObject(parent) {
    m_.setToIdentity();
  }

  /**
   * @brief Constructs a valid QRosTransformStamped from a ROS message.
   * @param tf      Source ROS TransformStamped message.
   * @param parent  Optional Qt parent.
   * @return Heap-allocated object to be owned by QML.
   */
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

  /**
   * @brief Constructs an invalid sentinel object (lookup failed).
   * @param parent  Optional Qt parent.
   * @return Heap-allocated object to be owned by QML; `valid` is false.
   */
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

  /// Sets the stamp in seconds and keeps the QDateTime property in sync.
  void setStampSec(double s) {
    if (qFuzzyCompare(stamp_sec_, s)) return;
    stamp_sec_ = s;
    // Keep QDateTime in sync (UTC)
    stamp_ = QDateTime::fromMSecsSinceEpoch(static_cast<qint64>(stamp_sec_ * 1000.0), Qt::UTC);
    emit stampSecChanged();
    emit stampChanged();
  }

  /// Sets the stamp as a QDateTime and keeps the seconds property in sync.
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
