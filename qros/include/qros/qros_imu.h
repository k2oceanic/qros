#pragma once

/**
 * @file qros_imu.h
 * @brief Publisher and subscriber for sensor_msgs/msg/Imu.
 */

#include "qros_subscriber.h"
#include "qros_publisher.h"
#include <sensor_msgs/msg/imu.hpp>
#include <QString>
#include <QQuaternion>
#include <QVector3D>
#include <QDateTime>

QROS_NS_HEAD

/**
 * @brief Publishes `sensor_msgs/Imu` messages.
 *
 * Exposes orientation (quaternion), angular velocity, linear acceleration,
 * frame ID, and timestamp as writable QML properties.  All fields use the
 * single `imuChanged` signal.
 *
 * ### QML usage
 * @code{.qml}
 * QRosImuPublisher {
 *     node:        applicationNode
 *     topic:       "/imu/raw"
 *     orientation: Qt.quaternion(1, 0, 0, 0)
 *     frameId:     "imu_link"
 * }
 * @endcode
 */
class QRosImuPublisher : public QRosPublisher {
  Q_OBJECT
public:
  /// Orientation as a quaternion (w, x, y, z).
  Q_PROPERTY(QQuaternion orientation READ getOrientation WRITE setOrientation NOTIFY imuChanged)
  /// Angular velocity in rad/s (x, y, z).
  Q_PROPERTY(QVector3D angularVelocity READ getAngularVelocity WRITE setAngularVelocity NOTIFY imuChanged)
  /// Linear acceleration in m/s² (x, y, z).
  Q_PROPERTY(QVector3D linearAcceleration READ getLinearAcceleration WRITE setLinearAcceleration NOTIFY imuChanged)
  /// Coordinate frame ID (e.g. "imu_link").
  Q_PROPERTY(QString frameId READ getFrameId WRITE setFrameId NOTIFY imuChanged)
  /// Message header timestamp.
  Q_PROPERTY(QDateTime timestamp READ getTimestamp WRITE setTimestamp NOTIFY imuChanged)

public slots:
  QQuaternion getOrientation() {
    auto orientation = publisher_.msgBuffer().orientation;
    return QQuaternion(orientation.w, orientation.x, orientation.y, orientation.z);
  }
  void setOrientation(QQuaternion ori) {
    publisher_.msgBuffer().orientation.x = ori.x();
    publisher_.msgBuffer().orientation.y = ori.y();
    publisher_.msgBuffer().orientation.z = ori.z();
    publisher_.msgBuffer().orientation.w = ori.scalar();
    emit imuChanged();
  }

  QVector3D getAngularVelocity() {
    auto angular = publisher_.msgBuffer().angular_velocity;
    return QVector3D(angular.x, angular.y, angular.z);
  }
  void setAngularVelocity(QVector3D angVel) {
    publisher_.msgBuffer().angular_velocity.x = angVel.x();
    publisher_.msgBuffer().angular_velocity.y = angVel.y();
    publisher_.msgBuffer().angular_velocity.z = angVel.z();
    emit imuChanged();
  }

  QVector3D getLinearAcceleration() {
    auto linear = publisher_.msgBuffer().linear_acceleration;
    return QVector3D(linear.x, linear.y, linear.z);
  }
  void setLinearAcceleration(QVector3D linAcc) {
    publisher_.msgBuffer().linear_acceleration.x = linAcc.x();
    publisher_.msgBuffer().linear_acceleration.y = linAcc.y();
    publisher_.msgBuffer().linear_acceleration.z = linAcc.z();
    emit imuChanged();
  }

  QString getFrameId() {
    return QString::fromStdString(publisher_.msgBuffer().header.frame_id);
  }

  void setFrameId(QString frameId) {
    publisher_.msgBuffer().header.frame_id = frameId.toStdString();
    emit imuChanged();
  }

  QDateTime getTimestamp() {
    qint64 totalMSecs = static_cast<qint64>(publisher_.msgBuffer().header.stamp.sec) * 1000LL +
                        static_cast<qint64>(publisher_.msgBuffer().header.stamp.nanosec) / 1000000LL;
    return QDateTime::fromMSecsSinceEpoch(totalMSecs);
  }

  void setTimestamp(QDateTime timestamp) {
    publisher_.msgBuffer().header.stamp.sec = timestamp.toSecsSinceEpoch();
    publisher_.msgBuffer().header.stamp.nanosec = (timestamp.toMSecsSinceEpoch() % 1000) * 1000000;
    emit imuChanged();
  }

signals:
  void imuChanged();

protected:
  QRosPublisherInterface* interfacePtr() { return &publisher_; }
  QRosTypedPublisher<sensor_msgs::msg::Imu> publisher_;
};

/**
 * @brief Subscribes to `sensor_msgs/Imu` messages.
 *
 * Exposes orientation, angular velocity, linear acceleration, frame ID,
 * and timestamp as read-only QML properties.
 *
 * ### QML usage
 * @code{.qml}
 * QRosImuSubscriber {
 *     node:  applicationNode
 *     topic: "/imu/data"
 *     onImuChanged: attitudeIndicator.quaternion = orientation
 * }
 * @endcode
 */
class QRosImuSubscriber : public QRosSubscriber {
  Q_OBJECT
public:
  /// Last received orientation quaternion (w, x, y, z).
  Q_PROPERTY(QQuaternion orientation READ getOrientation NOTIFY imuChanged)
  /// Last received angular velocity in rad/s.
  Q_PROPERTY(QVector3D angularVelocity READ getAngularVelocity NOTIFY imuChanged)
  /// Last received linear acceleration in m/s².
  Q_PROPERTY(QVector3D linearAcceleration READ getLinearAcceleration NOTIFY imuChanged)
  /// Frame ID from the last received message header.
  Q_PROPERTY(QString frameId READ getFrameId NOTIFY imuChanged)
  /// Timestamp from the last received message header.
  Q_PROPERTY(QDateTime timestamp READ getTimestamp NOTIFY imuChanged)

public slots:
  QQuaternion getOrientation() {
    auto orientation = subscriber_.msgBuffer().orientation;
    return QQuaternion(orientation.w, orientation.x, orientation.y, orientation.z);
  }

  QVector3D getAngularVelocity() {
    auto angular = subscriber_.msgBuffer().angular_velocity;
    return QVector3D(angular.x, angular.y, angular.z);
  }

  QVector3D getLinearAcceleration() {
    auto linear = subscriber_.msgBuffer().linear_acceleration;
    return QVector3D(linear.x, linear.y, linear.z);
  }

  QString getFrameId() {
    return QString::fromStdString(subscriber_.msgBuffer().header.frame_id);
  }

  QDateTime getTimestamp() {
    qint64 totalMSecs = static_cast<qint64>(subscriber_.msgBuffer().header.stamp.sec) * 1000LL +
                        static_cast<qint64>(subscriber_.msgBuffer().header.stamp.nanosec) / 1000000LL;
    return QDateTime::fromMSecsSinceEpoch(totalMSecs);
  }

signals:
  void imuChanged();

protected:
  void onMsgReceived() override {
    emit imuChanged();
  }

private:
  QRosSubscriberInterface* interfacePtr() { return &subscriber_; }
  QRosTypedSubscriber<sensor_msgs::msg::Imu> subscriber_;
};

QROS_NS_FOOT
