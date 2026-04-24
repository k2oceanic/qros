#pragma once

/**
 * @file qros_tf_buffer.h
 * @brief QML-exposed wrapper around a tf2_ros::Buffer for coordinate-frame transforms.
 */

#include "qros_object.h"
#include "qros_transform_stamped.h"

#include <mutex>
#include <memory>
#include <QStringList>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/time.h>

QROS_NS_HEAD

/**
 * @brief QML-exposed TF2 buffer that provides coordinate-frame lookups.
 *
 * QRosTfBuffer wraps a tf2_ros::Buffer and its associated TransformListener.
 * It does **not** spin its own thread — the caller's QRosNode timer drives
 * the executor, which processes incoming TF data.
 *
 * Set the `node` property to initialise the buffer.  After that, call
 * canTransform() / lookupTransform() from QML bindings or event handlers.
 * refreshFrames() updates the `frames` list for use in dropdown menus.
 *
 * ### QML usage
 * @code{.qml}
 * QRosTfBuffer {
 *     id:   tfBuffer
 *     node: applicationNode
 * }
 * Button {
 *     onClicked: {
 *         var tf = tfBuffer.lookupTransform("base_link", "camera_link")
 *         if (tf.valid) console.log(tf.translation)
 *     }
 * }
 * @endcode
 */
class QRosTfBuffer : public QRosObject {
  Q_OBJECT
public:
  QRosTfBuffer();
  ~QRosTfBuffer();

  /// Live list of all frame names visible in the TF tree.  Call refreshFrames() to update.
  Q_PROPERTY(QStringList frames READ frames NOTIFY framesChanged)

  /**
   * @brief Returns true if a transform from @p source_frame to @p target_frame is available.
   * @param target_frame  Destination coordinate frame.
   * @param source_frame  Source coordinate frame.
   */
  Q_INVOKABLE bool canTransform(const QString &target_frame,
                                const QString &source_frame) const;

  /**
   * @brief Looks up the transform from @p source_frame to @p target_frame at the latest time.
   * @param target_frame  Destination frame.
   * @param source_frame  Source frame.
   * @param time_sec      Lookup time in seconds (0.0 = latest).
   * @return A new QRosTransformStamped owned by QML, or an invalid object on failure.
   */
  Q_INVOKABLE QRosTransformStamped* lookupTransform(const QString &target_frame,
                                                    const QString &source_frame,
                                                    double time_sec = 0.0) const;

  /**
   * @brief Full-form transform lookup between two independent time points.
   * @param target_frame      Destination frame.
   * @param target_time_sec   Time at which the target frame is sampled.
   * @param source_frame      Source frame.
   * @param source_time_sec   Time at which the source frame is sampled.
   * @param fixed_frame       The fixed world frame used to bridge the two times.
   * @return A new QRosTransformStamped owned by QML, or invalid on failure.
   */
  Q_INVOKABLE QRosTransformStamped* lookupTransformFull(const QString &target_frame, double target_time_sec,
                                                        const QString &source_frame, double source_time_sec,
                                                        const QString &fixed_frame) const;

  /**
   * @brief Returns true if @p frame exists in the current TF tree.
   * @param frame  Frame name to check.
   */
  Q_INVOKABLE bool frameExists(const QString &frame) const;

  /**
   * @brief Refreshes the `frames` list from the current TF buffer state.
   *
   * Call this from QML (e.g. a Timer or a button) whenever the dropdown
   * list of available frames needs updating.
   */
  Q_INVOKABLE void refreshFrames();

  /// Returns the cached frame name list.
  QStringList frames() const {
    return frames_;
  }

signals:
  /// Emitted when the buffer becomes ready or unavailable.
  void readyChanged(bool ready);
  /// Emitted when the frames list is updated by refreshFrames().
  void framesChanged();

private slots:
  void onNodeChanged();

private:
  void ensureBufferLocked() const;

  mutable std::shared_ptr<tf2_ros::Buffer> buffer_;
  mutable std::unique_ptr<tf2_ros::TransformListener> listener_;
  bool ready_ = false;

  QStringList frames_;
};

QROS_NS_FOOT
