#pragma once

#include "qros_subscriber.h"
#include "qros_publisher.h"
#include <propulsion_interfaces/msg/thrust_stamped.hpp>
#include <QString>
#include <QDateTime>

QROS_NS_HEAD

    class QRosThrustStampedPublisher : public QRosPublisher{
  Q_OBJECT
public:
  Q_PROPERTY(float scale READ getScale WRITE setScale NOTIFY thrustChanged)
  Q_PROPERTY(float proportionalValue READ getProportionalValue WRITE setProportionalValue NOTIFY thrustChanged)
  Q_PROPERTY(QString frameId READ getFrameId WRITE setFrameId NOTIFY thrustChanged)
  Q_PROPERTY(QDateTime timestamp READ getTimestamp WRITE setTimestamp NOTIFY thrustChanged)

public slots:
  float getScale(){
    return publisher_.msgBuffer().thrust.scale;
  }
  void setScale(float scl){
    publisher_.msgBuffer().thrust.scale = scl;
    emit thrustChanged();
  }

  float getProportionalValue(){
    return publisher_.msgBuffer().thrust.proportional_value;
  }
  void setProportionalValue(float value){
    publisher_.msgBuffer().thrust.proportional_value = value;
    emit thrustChanged();
  }

  QString getFrameId() {
    return QString::fromStdString(publisher_.msgBuffer().header.frame_id);
  }

  void setFrameId(QString frameId) {
    publisher_.msgBuffer().header.frame_id = frameId.toStdString();
    emit thrustChanged();
  }

  QDateTime getTimestamp() {
    qint64 totalMSecs = static_cast<qint64>(publisher_.msgBuffer().header.stamp.sec) * 1000LL +
                        static_cast<qint64>(publisher_.msgBuffer().header.stamp.nanosec) / 1000000LL;
    return QDateTime::fromMSecsSinceEpoch(totalMSecs);
  }

  void setTimestamp(QDateTime timestamp) {
    publisher_.msgBuffer().header.stamp.sec = timestamp.toSecsSinceEpoch();
    publisher_.msgBuffer().header.stamp.nanosec = (timestamp.toMSecsSinceEpoch() % 1000) * 1000000;
    emit thrustChanged();
  }

signals:
  void thrustChanged();

protected:
  QRosPublisherInterface* interfacePtr(){return &publisher_;}
  QRosTypedPublisher<propulsion_interfaces::msg::ThrustStamped> publisher_;
};


class QRosThrustStampedSubscriber : public QRosSubscriber{
  Q_OBJECT
public:
  Q_PROPERTY(float scale READ getScale NOTIFY thrustChanged)
  Q_PROPERTY(float proportionalValue READ getProportionalValue NOTIFY thrustChanged)
  Q_PROPERTY(QString frameId READ getFrameId NOTIFY thrustChanged)
  Q_PROPERTY(QDateTime timestamp READ getTimestamp NOTIFY thrustChanged)

public slots:
  float getScale() {
    return subscriber_.msgBuffer().thrust.scale;
  }

  float getProportionalValue() {
    return subscriber_.msgBuffer().thrust.proportional_value;
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
  void thrustChanged();

protected:
  void onMsgReceived() override{
    emit thrustChanged();
  }

private:
  QRosSubscriberInterface* interfacePtr(){return &subscriber_;}
  QRosTypedSubscriber<propulsion_interfaces::msg::ThrustStamped> subscriber_;
};

QROS_NS_FOOT
