#pragma once

#include "qros_subscriber.h"
#include "qros_publisher.h"
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <QString>
#include <QVector3D>
#include <QDateTime>

QROS_NS_HEAD

class QRosWrenchStampedPublisher : public QRosPublisher{
  Q_OBJECT
public:
  Q_PROPERTY(QVector3D force READ getForce WRITE setForce NOTIFY wrenchChanged)
  Q_PROPERTY(QVector3D torque READ getTorque WRITE setTorque NOTIFY wrenchChanged)
  Q_PROPERTY(QString frameId READ getFrameId WRITE setFrameId NOTIFY wrenchChanged)
  Q_PROPERTY(QDateTime timestamp READ getTimestamp WRITE setTimestamp NOTIFY wrenchChanged)

public slots:
  QVector3D getForce(){
    auto force = publisher_.msgBuffer().wrench.force;
    return QVector3D(force.x, force.y, force.z);
  }
  void setForce(QVector3D frc){
    publisher_.msgBuffer().wrench.force.x = frc.x();
    publisher_.msgBuffer().wrench.force.y = frc.y();
    publisher_.msgBuffer().wrench.force.z = frc.z();
    emit wrenchChanged();
  }

  QVector3D getTorque(){
    auto torque = publisher_.msgBuffer().wrench.torque;
    return QVector3D(torque.x, torque.y, torque.z);
  }
  void setTorque(QVector3D trq){
    publisher_.msgBuffer().wrench.torque.x = trq.x();
    publisher_.msgBuffer().wrench.torque.y = trq.y();
    publisher_.msgBuffer().wrench.torque.z = trq.z();
    emit wrenchChanged();
  }

  QString getFrameId() {
    return QString::fromStdString(publisher_.msgBuffer().header.frame_id);
  }

  void setFrameId(QString frameId) {
    publisher_.msgBuffer().header.frame_id = frameId.toStdString();
    emit wrenchChanged();
  }

  QDateTime getTimestamp() {
    qint64 totalMSecs = static_cast<qint64>(publisher_.msgBuffer().header.stamp.sec) * 1000LL +
                        static_cast<qint64>(publisher_.msgBuffer().header.stamp.nanosec) / 1000000LL;
    return QDateTime::fromMSecsSinceEpoch(totalMSecs);
  }

  void setTimestamp(QDateTime timestamp) {
    publisher_.msgBuffer().header.stamp.sec = timestamp.toSecsSinceEpoch();
    publisher_.msgBuffer().header.stamp.nanosec = (timestamp.toMSecsSinceEpoch() % 1000) * 1000000;
    emit wrenchChanged();
  }

signals:
  void wrenchChanged();

protected:
  QRosPublisherInterface* interfacePtr(){return &publisher_;}
  QRosTypedPublisher<geometry_msgs::msg::WrenchStamped> publisher_;
};


class QRosWrenchStampedSubscriber : public QRosSubscriber{
  Q_OBJECT
public:
  Q_PROPERTY(QVector3D force READ getForce NOTIFY wrenchChanged)
  Q_PROPERTY(QVector3D torque READ getTorque NOTIFY wrenchChanged)
  Q_PROPERTY(QString frameId READ getFrameId NOTIFY wrenchChanged)
  Q_PROPERTY(QDateTime timestamp READ getTimestamp NOTIFY wrenchChanged)

public slots:
  QVector3D getForce() {
    auto force = subscriber_.msgBuffer().wrench.force;
    return QVector3D(force.x, force.y, force.z);
  }

  QVector3D getTorque() {
    auto torque = subscriber_.msgBuffer().wrench.torque;
    return QVector3D(torque.x, torque.y, torque.z);
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
  void wrenchChanged();

protected:
  void onMsgReceived() override{
    emit wrenchChanged();
  }

private:
  QRosSubscriberInterface* interfacePtr(){return &subscriber_;}
  QRosTypedSubscriber<geometry_msgs::msg::WrenchStamped> subscriber_;
};

QROS_NS_FOOT
