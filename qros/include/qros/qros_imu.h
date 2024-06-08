#pragma once

#include "qros_subscriber.h"
#include "qros_publisher.h"
#include <sensor_msgs/msg/imu.hpp>
#include <QString>
#include <QQuaternion>
#include <QVector3D>
#include <QDateTime>

QROS_NS_HEAD

    class QRosImuPublisher : public QRosPublisher {
  Q_OBJECT
public:
  // Orientation properties
  Q_PROPERTY(QQuaternion orientation READ getOrientation WRITE setOrientation NOTIFY imuChanged)
  Q_PROPERTY(QVector3D angularVelocity READ getAngularVelocity WRITE setAngularVelocity NOTIFY imuChanged)
  Q_PROPERTY(QVector3D linearAcceleration READ getLinearAcceleration WRITE setLinearAcceleration NOTIFY imuChanged)
  // Frame ID and timestamp
  Q_PROPERTY(QString frameId READ getFrameId WRITE setFrameId NOTIFY imuChanged)
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

class QRosImuSubscriber : public QRosSubscriber {
  Q_OBJECT
public:
  Q_PROPERTY(QQuaternion orientation READ getOrientation NOTIFY imuChanged)
  Q_PROPERTY(QVector3D angularVelocity READ getAngularVelocity NOTIFY imuChanged)
  Q_PROPERTY(QVector3D linearAcceleration READ getLinearAcceleration NOTIFY imuChanged)
  Q_PROPERTY(QString frameId READ getFrameId NOTIFY imuChanged)
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
