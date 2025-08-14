#include "qros_tf_buffer.h"
#include "qros_node.h"

#include <QQmlEngine>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

QROS_NS_HEAD

QRosTfBuffer::QRosTfBuffer() {
  QObject::connect(this, &QRosTfBuffer::nodeChanged, this, &QRosTfBuffer::onNodeChanged);
}

QRosTfBuffer::~QRosTfBuffer() {
  listener_.reset();
  buffer_.reset();
}

void QRosTfBuffer::onNodeChanged() {
  listener_.reset();
  buffer_.reset();
  bool prev_ready = ready_;
  ready_ = false;
  frames_.clear();
  emit framesChanged();

  auto ros_node = getRosNode();
  if (!ros_node) {
    if (prev_ready != ready_) emit readyChanged(ready_);
    return;
  }

  // No dedicated thread or timer interface — we'll only use zero timeouts.
  buffer_ = std::make_shared<tf2_ros::Buffer>(ros_node->get_clock());

  try {
    listener_ = std::make_unique<tf2_ros::TransformListener>(*buffer_, ros_node, /*spin_thread=*/false);
    ready_ = true;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(ros_node->get_logger(), "QRosTfBuffer: failed to construct TransformListener: %s", e.what());
    listener_.reset();
    buffer_.reset();
    ready_ = false;
  }
  if (prev_ready != ready_) emit readyChanged(ready_);
}

bool QRosTfBuffer::canTransform(const QString &target_frame, const QString &source_frame) const {
  if (!buffer_) return false;
  try {
    // Non-blocking (no timeout)
    return buffer_->canTransform(target_frame.toStdString(), source_frame.toStdString(),
                                 tf2::TimePointZero);
  } catch (...) { return false; }
}

QRosTransformStamped* QRosTfBuffer::lookupTransform(const QString &target_frame,
                                                    const QString &source_frame,
                                                    double time_sec) const {
  if (!buffer_) {
    auto *obj = QRosTransformStamped::invalid();
    QQmlEngine::setObjectOwnership(obj, QQmlEngine::JavaScriptOwnership);
    return obj;
  }
  try {
    const tf2::TimePoint t = (time_sec <= 0.0) ? tf2::TimePointZero : tf2::timeFromSec(time_sec);
    auto tf = buffer_->lookupTransform(target_frame.toStdString(), source_frame.toStdString(), t);
    auto *obj = QRosTransformStamped::fromMsg(tf);
    QQmlEngine::setObjectOwnership(obj, QQmlEngine::JavaScriptOwnership);
    return obj;
  } catch (...) {
    auto *obj = QRosTransformStamped::invalid();
    QQmlEngine::setObjectOwnership(obj, QQmlEngine::JavaScriptOwnership);
    return obj;
  }
}

QRosTransformStamped* QRosTfBuffer::lookupTransformFull(const QString &target_frame,
                                                        double target_time_sec,
                                                        const QString &source_frame,
                                                        double source_time_sec,
                                                        const QString &fixed_frame) const {
  if (!buffer_) {
    auto *obj = QRosTransformStamped::invalid();
    QQmlEngine::setObjectOwnership(obj, QQmlEngine::JavaScriptOwnership);
    return obj;
  }
  try {
    const tf2::TimePoint tt = (target_time_sec <= 0.0) ? tf2::TimePointZero : tf2::timeFromSec(target_time_sec);
    const tf2::TimePoint ts = (source_time_sec <= 0.0) ? tf2::TimePointZero : tf2::timeFromSec(source_time_sec);

    auto tf = buffer_->lookupTransform(target_frame.toStdString(), tt,
                                       source_frame.toStdString(), ts,
                                       fixed_frame.toStdString());
    auto *obj = QRosTransformStamped::fromMsg(tf);
    QQmlEngine::setObjectOwnership(obj, QQmlEngine::JavaScriptOwnership);
    return obj;
  } catch (...) {
    auto *obj = QRosTransformStamped::invalid();
    QQmlEngine::setObjectOwnership(obj, QQmlEngine::JavaScriptOwnership);
    return obj;
  }
}

bool QRosTfBuffer::frameExists(const QString &frame) const {
  if (!buffer_) return false;
  try {
    return buffer_->canTransform(frame.toStdString(), frame.toStdString(),
                                 tf2::TimePointZero, tf2::Duration(0));
  } catch (...) { return false; }
}

void QRosTfBuffer::refreshFrames() {
  QStringList new_list;
  if (buffer_) {
    // tf2_ros::Buffer inherits tf2::BufferCore, so getAllFrameNames() is available
    auto names = buffer_->getAllFrameNames(); // std::vector<std::string>
    new_list.reserve(static_cast<int>(names.size()));
    for (const auto &s : names) new_list.push_back(QString::fromStdString(s));
    // Optional: sort for nicer UX
    std::sort(new_list.begin(), new_list.end(), [](const QString &a, const QString &b){ return a.toLower() < b.toLower(); });
  }
  if (new_list != frames_) {
    frames_ = new_list;
    emit framesChanged();
  }
}

QROS_NS_FOOT
