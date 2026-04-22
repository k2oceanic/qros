#pragma once

/**
 * @file qros_odometry.h
 * @brief Publisher and subscriber for nav_msgs/msg/Odometry.
 */

#include "qros_subscriber.h"
#include "qros_publisher.h"
#include <nav_msgs/msg/odometry.hpp>
#include <QString>
#include <QQuaternion>
#include <QVector3D>
#include <QDateTime>

QROS_NS_HEAD

/**
 * @brief Publishes `nav_msgs/Odometry` messages.
 *
 * Exposes pose (position + orientation) and twist (linear + angular velocity)
 * as writable QML properties.
 *
 * ### QML usage
 * @code{.qml}
 * QRosOdometryPublisher {
 *     node:           applicationNode
 *     topic:          "/odom_override"
 *     position:       Qt.vector3d(x, y, z)
 *     orientation:    Qt.quaternion(w, x, y, z)
 *     linearVelocity: Qt.vector3d(vx, vy, 0)
 *     frameId:        "odom"
 * }
 * @endcode
 */
class QRosOdometryPublisher : public QRosPublisher {
  Q_OBJECT
public:
  /// Pose position (x, y, z) in metres.
  Q_PROPERTY(QVector3D position READ getPosition WRITE setPosition NOTIFY odometryChanged)
  /// Pose orientation quaternion (w, x, y, z).
  Q_PROPERTY(QQuaternion orientation READ getOrientation WRITE setOrientation NOTIFY odometryChanged)
  /// Linear velocity (x, y, z) in m/s.
  Q_PROPERTY(QVector3D linearVelocity READ getLinearVelocity WRITE setLinearVelocity NOTIFY odometryChanged)
  /// Angular velocity (roll, pitch, yaw) in rad/s.
  Q_PROPERTY(QVector3D angularVelocity READ getAngularVelocity WRITE setAngularVelocity NOTIFY odometryChanged)
  /// Coordinate frame ID (e.g. "odom").
  Q_PROPERTY(QString frameId READ getFrameId WRITE setFrameId NOTIFY odometryChanged)
  /// Message header timestamp.
  Q_PROPERTY(QDateTime timestamp READ getTimestamp WRITE setTimestamp NOTIFY odometryChanged)

public slots:
  QVector3D getPosition() {
    auto position = publisher_.msgBuffer().pose.pose.position;
    return QVector3D(position.x, position.y, position.z);
  }
  void setPosition(QVector3D pos) {
    publisher_.msgBuffer().pose.pose.position.x = pos.x();
    publisher_.msgBuffer().pose.pose.position.y = pos.y();
    publisher_.msgBuffer().pose.pose.position.z = pos.z();
    emit odometryChanged();
  }

  QQuaternion getOrientation() {
    auto orientation = publisher_.msgBuffer().pose.pose.orientation;
    return QQuaternion(orientation.w, orientation.x, orientation.y, orientation.z);
  }
  void setOrientation(QQuaternion ori) {
    publisher_.msgBuffer().pose.pose.orientation.x = ori.x();
    publisher_.msgBuffer().pose.pose.orientation.y = ori.y();
    publisher_.msgBuffer().pose.pose.orientation.z = ori.z();
    publisher_.msgBuffer().pose.pose.orientation.w = ori.scalar();
    emit odometryChanged();
  }

  QVector3D getLinearVelocity() {
    auto linear = publisher_.msgBuffer().twist.twist.linear;
    return QVector3D(linear.x, linear.y, linear.z);
  }
  void setLinearVelocity(QVector3D linVel) {
    publisher_.msgBuffer().twist.twist.linear.x = linVel.x();
    publisher_.msgBuffer().twist.twist.linear.y = linVel.y();
    publisher_.msgBuffer().twist.twist.linear.z = linVel.z();
    emit odometryChanged();
  }

  QVector3D getAngularVelocity() {
    auto angular = publisher_.msgBuffer().twist.twist.angular;
    return QVector3D(angular.x, angular.y, angular.z);
  }
  void setAngularVelocity(QVector3D angVel) {
    publisher_.msgBuffer().twist.twist.angular.x = angVel.x();
    publisher_.msgBuffer().twist.twist.angular.y = angVel.y();
    publisher_.msgBuffer().twist.twist.angular.z = angVel.z();
    emit odometryChanged();
  }

  QString getFrameId() {
    return QString::fromStdString(publisher_.msgBuffer().header.frame_id);
  }

  void setFrameId(QString frameId) {
    publisher_.msgBuffer().header.frame_id = frameId.toStdString();
    emit odometryChanged();
  }

  QDateTime getTimestamp() {
    qint64 totalMSecs = static_cast<qint64>(publisher_.msgBuffer().header.stamp.sec) * 1000LL +
                        static_cast<qint64>(publisher_.msgBuffer().header.stamp.nanosec) / 1000000LL;
    return QDateTime::fromMSecsSinceEpoch(totalMSecs);
  }

  void setTimestamp(QDateTime timestamp) {
    publisher_.msgBuffer().header.stamp.sec = timestamp.toSecsSinceEpoch();
    publisher_.msgBuffer().header.stamp.nanosec = (timestamp.toMSecsSinceEpoch() % 1000) * 1000000;
    emit odometryChanged();
  }

signals:
  void odometryChanged();

protected:
  QRosPublisherInterface* interfacePtr() { return &publisher_; }
  QRosTypedPublisher<nav_msgs::msg::Odometry> publisher_;
};

/**
 * @brief Subscribes to `nav_msgs/Odometry` messages.
 *
 * ### QML usage
 * @code{.qml}
 * QRosOdometrySubscriber {
 *     node:  applicationNode
 *     topic: "/odom"
 *     onOdometryChanged: {
 *         positionDisplay.x = position.x
 *         positionDisplay.y = position.y
 *         headingDisplay.value = Qt.eulerAngles(orientation).z
 *     }
 * }
 * @endcode
 */
class QRosOdometrySubscriber : public QRosSubscriber {
  Q_OBJECT
public:
  /// Pose position from the last received message.
  Q_PROPERTY(QVector3D position READ getPosition NOTIFY odometryChanged)
  /// Pose orientation quaternion from the last received message.
  Q_PROPERTY(QQuaternion orientation READ getOrientation NOTIFY odometryChanged)
  /// Linear velocity from the last received message.
  Q_PROPERTY(QVector3D linearVelocity READ getLinearVelocity NOTIFY odometryChanged)
  /// Angular velocity from the last received message.
  Q_PROPERTY(QVector3D angularVelocity READ getAngularVelocity NOTIFY odometryChanged)
  /// Frame ID from the last received message header.
  Q_PROPERTY(QString frameId READ getFrameId NOTIFY odometryChanged)
  /// Child frame ID from the last received message.
  Q_PROPERTY(QString childFrameId READ getChildFrameId NOTIFY odometryChanged)
  /// Timestamp from the last received message header.
  Q_PROPERTY(QDateTime timestamp READ getTimestamp NOTIFY odometryChanged)

public slots:
  QVector3D getPosition() {
    auto position = subscriber_.msgBuffer().pose.pose.position;
    return QVector3D(position.x, position.y, position.z);
  }

  QQuaternion getOrientation() {
    auto orientation = subscriber_.msgBuffer().pose.pose.orientation;
    return QQuaternion(orientation.w, orientation.x, orientation.y, orientation.z);
  }

  QVector3D getLinearVelocity() {
    auto linear = subscriber_.msgBuffer().twist.twist.linear;
    return QVector3D(linear.x, linear.y, linear.z);
  }

  QVector3D getAngularVelocity() {
    auto angular = subscriber_.msgBuffer().twist.twist.angular;
    return QVector3D(angular.x, angular.y, angular.z);
  }

  QString getFrameId() {
    return QString::fromStdString(subscriber_.msgBuffer().header.frame_id);
  }

  QString getChildFrameId() {
    return QString::fromStdString(subscriber_.msgBuffer().child_frame_id);
  }

  QDateTime getTimestamp() {
    qint64 totalMSecs = static_cast<qint64>(subscriber_.msgBuffer().header.stamp.sec) * 1000LL +
                        static_cast<qint64>(subscriber_.msgBuffer().header.stamp.nanosec) / 1000000LL;
    return QDateTime::fromMSecsSinceEpoch(totalMSecs);
  }

signals:
  void odometryChanged();

protected:
  void onMsgReceived() override {
    emit odometryChanged();
  }

private:
  QRosSubscriberInterface* interfacePtr() { return &subscriber_; }
  QRosTypedSubscriber<nav_msgs::msg::Odometry> subscriber_;
};

QROS_NS_FOOT
