#pragma once

/**
 * @file qros_twist_stamped.h
 * @brief Publisher and subscriber for geometry_msgs/msg/TwistStamped.
 */

#include "qros_subscriber.h"
#include "qros_publisher.h"
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <QString>
#include <QVector3D>
#include <QDateTime>

QROS_NS_HEAD

/**
 * @brief Publishes `geometry_msgs/TwistStamped` messages.
 *
 * ### QML usage
 * @code{.qml}
 * QRosTwistStampedPublisher {
 *     node:    applicationNode
 *     topic:   "/cmd_vel"
 *     linear:  Qt.vector3d(forwardSpeed, 0, 0)
 *     angular: Qt.vector3d(0, 0, yawRate)
 *     frameId: "base_link"
 * }
 * @endcode
 */
class QRosTwistStampedPublisher : public QRosPublisher{
  Q_OBJECT
public:
  /// Linear velocity (x, y, z) in m/s.
  Q_PROPERTY(QVector3D linear READ getLinear WRITE setLinear NOTIFY twistChanged)
  /// Angular velocity (roll, pitch, yaw) in rad/s.
  Q_PROPERTY(QVector3D angular READ getAngular WRITE setAngular NOTIFY twistChanged)
  /// Coordinate frame ID.
  Q_PROPERTY(QString frameId READ getFrameId WRITE setFrameId NOTIFY twistChanged)
  /// Message header timestamp.
  Q_PROPERTY(QDateTime timestamp READ getTimestamp WRITE setTimestamp NOTIFY twistChanged)

public slots:
  QVector3D getLinear(){
    auto linear = publisher_.msgBuffer().twist.linear;
    return QVector3D(linear.x, linear.y, linear.z);
  }
  void setLinear(QVector3D lin){
    publisher_.msgBuffer().twist.linear.x = lin.x();
    publisher_.msgBuffer().twist.linear.y = lin.y();
    publisher_.msgBuffer().twist.linear.z = lin.z();
    emit twistChanged();
  }

  QVector3D getAngular(){
    auto angular = publisher_.msgBuffer().twist.angular;
    return QVector3D(angular.x, angular.y, angular.z);
  }
  void setAngular(QVector3D ang){
    publisher_.msgBuffer().twist.angular.x = ang.x();
    publisher_.msgBuffer().twist.angular.y = ang.y();
    publisher_.msgBuffer().twist.angular.z = ang.z();
    emit twistChanged();
  }

  QString getFrameId() {
    return QString::fromStdString(publisher_.msgBuffer().header.frame_id);
  }

  void setFrameId(QString frameId) {
    publisher_.msgBuffer().header.frame_id = frameId.toStdString();
    emit twistChanged();
  }

  QDateTime getTimestamp() {
    qint64 totalMSecs = static_cast<qint64>(publisher_.msgBuffer().header.stamp.sec) * 1000LL +
                        static_cast<qint64>(publisher_.msgBuffer().header.stamp.nanosec) / 1000000LL;
    return QDateTime::fromMSecsSinceEpoch(totalMSecs);
  }

  void setTimestamp(QDateTime timestamp) {
    publisher_.msgBuffer().header.stamp.sec = timestamp.toSecsSinceEpoch();
    publisher_.msgBuffer().header.stamp.nanosec = (timestamp.toMSecsSinceEpoch() % 1000) * 1000000;
    emit twistChanged();
  }

signals:
  void twistChanged();

protected:
  QRosPublisherInterface* interfacePtr(){return &publisher_;}
  QRosTypedPublisher<geometry_msgs::msg::TwistStamped> publisher_;
};



/**
 * @brief Subscribes to `geometry_msgs/TwistStamped` messages.
 *
 * ### QML usage
 * @code{.qml}
 * QRosTwistStampedSubscriber {
 *     node:  applicationNode
 *     topic: "/odom/twist"
 *     onTwistChanged: speedDisplay.value = linear.x
 * }
 * @endcode
 */
class QRosTwistStampedSubscriber : public QRosSubscriber{
  Q_OBJECT
public:
  /// Linear velocity from the last received message.
  Q_PROPERTY(QVector3D linear READ getLinear NOTIFY twistChanged)
  /// Angular velocity from the last received message.
  Q_PROPERTY(QVector3D angular READ getAngular NOTIFY twistChanged)
  /// Frame ID from the last received message header.
  Q_PROPERTY(QString frameId READ getFrameId NOTIFY twistChanged)
  /// Timestamp from the last received message header.
  Q_PROPERTY(QDateTime timestamp READ getTimestamp NOTIFY twistChanged)

public slots:
  QVector3D getLinear() {
    auto linear = subscriber_.msgBuffer().twist.linear;
    return QVector3D(linear.x, linear.y, linear.z);
  }

  QVector3D getAngular() {
    auto angular = subscriber_.msgBuffer().twist.angular;
    return QVector3D(angular.x, angular.y, angular.z);
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
  void twistChanged();

protected:
  void onMsgReceived() override{
    emit twistChanged();
  }

private:
  QRosSubscriberInterface* interfacePtr(){return &subscriber_;}
  QRosTypedSubscriber<geometry_msgs::msg::TwistStamped> subscriber_;
};

QROS_NS_FOOT
