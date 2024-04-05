#pragma once

#include "qros_subscriber.h"
#include "qros_publisher.h"
#include <nav_msgs/msg/odometry.hpp>
#include <QString>
#include <QQuaternion>
#include <QVector3D>
#include <QDateTime>

QROS_NS_HEAD


class QRosOdometryPublisher : public QRosPublisher {
  Q_OBJECT
public:
  // Pose properties
  Q_PROPERTY(QVector3D position READ getPosition WRITE setPosition NOTIFY odometryChanged)
  Q_PROPERTY(QQuaternion orientation READ getOrientation WRITE setOrientation NOTIFY odometryChanged)
  // Twist properties
  Q_PROPERTY(QVector3D linearVelocity READ getLinearVelocity WRITE setLinearVelocity NOTIFY odometryChanged)
  Q_PROPERTY(QVector3D angularVelocity READ getAngularVelocity WRITE setAngularVelocity NOTIFY odometryChanged)
  // Frame ID and timestamp
  Q_PROPERTY(QString frameId READ getFrameId WRITE setFrameId NOTIFY odometryChanged)
  Q_PROPERTY(QDateTime timestamp READ getTimestamp WRITE setTimestamp NOTIFY odometryChanged)

public slots:
  // Pose methods
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

  // Twist methods
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

class QRosOdometrySubscriber : public QRosSubscriber {
  Q_OBJECT
public:
  Q_PROPERTY(QVector3D position READ getPosition NOTIFY odometryChanged)
  Q_PROPERTY(QQuaternion orientation READ getOrientation NOTIFY odometryChanged)
  Q_PROPERTY(QVector3D linearVelocity READ getLinearVelocity NOTIFY odometryChanged)
  Q_PROPERTY(QVector3D angularVelocity READ getAngularVelocity NOTIFY odometryChanged)
  Q_PROPERTY(QString frameId READ getFrameId NOTIFY odometryChanged)
  Q_PROPERTY(QString childFrameId READ getChildFrameId NOTIFY odometryChanged)

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
