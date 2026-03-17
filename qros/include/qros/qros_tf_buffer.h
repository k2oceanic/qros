#pragma once
#include "qros_object.h"
#include "qros_transform_stamped.h"

#include <mutex>
#include <memory>
#include <QStringList>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/time.h>

QROS_NS_HEAD

    // Never spins; caller must spin the QRosNode.
    class QRosTfBuffer : public QRosObject {
  Q_OBJECT
public:
  QRosTfBuffer();
  ~QRosTfBuffer();

  // --- New: frames list for QML dropdowns ---
  Q_PROPERTY(QStringList frames READ frames NOTIFY framesChanged)

  Q_INVOKABLE bool canTransform(const QString &target_frame,
                                const QString &source_frame) const;

  // Returns a new QRosTransformStamped* owned by QML (or an invalid object on failure)
  Q_INVOKABLE QRosTransformStamped* lookupTransform(const QString &target_frame,
                                                    const QString &source_frame,
                                                    double time_sec = 0.0) const;

  Q_INVOKABLE QRosTransformStamped* lookupTransformFull(const QString &target_frame, double target_time_sec,
                                                        const QString &source_frame, double source_time_sec,
                                                        const QString &fixed_frame) const;

  Q_INVOKABLE bool frameExists(const QString &frame) const;

  // --- New: refresh frames list from the buffer (call this from QML) ---
  Q_INVOKABLE void refreshFrames();

  // Getter for Q_PROPERTY
  QStringList frames() const {
    return frames_;
  }

signals:
  void readyChanged(bool ready);
  void framesChanged();   // Emitted when frames list updates

private slots:
  void onNodeChanged();

private:
  void ensureBufferLocked() const;

  mutable std::shared_ptr<tf2_ros::Buffer> buffer_;
  mutable std::unique_ptr<tf2_ros::TransformListener> listener_;
  bool ready_ = false;

  // Cached list of frame names for QML
  QStringList frames_;
};

QROS_NS_FOOT
