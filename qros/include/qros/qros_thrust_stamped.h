#pragma once

/**
 * @file qros_thrust_stamped.h
 * @brief Publisher and subscriber for propulsion_interfaces/msg/ThrustStamped.
 */

#include "qros_subscriber.h"
#include "qros_publisher.h"
#include <propulsion_interfaces/msg/thrust_stamped.hpp>
#include <QString>
#include <QDateTime>

QROS_NS_HEAD

/**
 * @brief Publishes `propulsion_interfaces/ThrustStamped` messages.
 *
 * Commands a single thruster by scale (max thrust in Newtons) and proportional
 * value (−1.0 to 1.0), with a header for frame ID and timestamp.
 *
 * ### QML usage
 * @code{.qml}
 * QRosThrustStampedPublisher {
 *     node:             applicationNode
 *     topic:            "/thruster_1/cmd"
 *     scale:            80.0
 *     proportionalValue: throttleSlider.value
 *     frameId:          "thruster_1_frame"
 * }
 * @endcode
 */
class QRosThrustStampedPublisher : public QRosPublisher{
  Q_OBJECT
public:
  /// Maximum thrust scale in Newtons.
  Q_PROPERTY(float scale READ getScale WRITE setScale NOTIFY thrustChanged)
  /// Proportional thrust command (−1.0 = full reverse, 1.0 = full forward).
  Q_PROPERTY(float proportionalValue READ getProportionalValue WRITE setProportionalValue NOTIFY thrustChanged)
  /// Coordinate frame ID.
  Q_PROPERTY(QString frameId READ getFrameId WRITE setFrameId NOTIFY thrustChanged)
  /// Message header timestamp.
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


/**
 * @brief Subscribes to `propulsion_interfaces/ThrustStamped` messages.
 *
 * ### QML usage
 * @code{.qml}
 * QRosThrustStampedSubscriber {
 *     node:  applicationNode
 *     topic: "/thruster_1/feedback"
 *     onThrustChanged: thrusterBar.value = proportionalValue
 * }
 * @endcode
 */
class QRosThrustStampedSubscriber : public QRosSubscriber{
  Q_OBJECT
public:
  /// Scale value from the last received message.
  Q_PROPERTY(float scale READ getScale NOTIFY thrustChanged)
  /// Proportional value from the last received message.
  Q_PROPERTY(float proportionalValue READ getProportionalValue NOTIFY thrustChanged)
  /// Frame ID from the last received message header.
  Q_PROPERTY(QString frameId READ getFrameId NOTIFY thrustChanged)
  /// Timestamp from the last received message header.
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
