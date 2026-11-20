#pragma once

/**
 * @file qros_range.h
 * @brief Publisher and subscriber for sensor_msgs/msg/Range.
 */

#include "qros_subscriber.h"
#include "qros_publisher.h"
#include <sensor_msgs/msg/range.hpp>
#include <QString>
#include <QDateTime>

QROS_NS_HEAD

/**
 * @brief Publishes `sensor_msgs/Range` messages.
 *
 * Suitable for sonar, ultrasonic, or IR range sensors.
 *
 * ### QML usage
 * @code{.qml}
 * QRosRangePublisher {
 *     node:     applicationNode
 *     topic:    "/sonar/range_override"
 *     range:    altimeterValue
 *     minRange: 0.5
 *     maxRange: 100.0
 * }
 * @endcode
 */
class QRosRangePublisher : public QRosPublisher{
  Q_OBJECT
public:
  /// Measured range in metres.
  Q_PROPERTY(float range READ getRange WRITE setRange NOTIFY rangeChanged)
  /// Sensor field of view in radians.
  Q_PROPERTY(float fieldOfView READ getFieldOfView WRITE setFieldOfView NOTIFY rangeChanged)
  /// Minimum valid range in metres.
  Q_PROPERTY(float minRange READ getMinRange WRITE setMinRange NOTIFY rangeChanged)
  /// Maximum valid range in metres.
  Q_PROPERTY(float maxRange READ getMaxRange WRITE setMaxRange NOTIFY rangeChanged)
  /// Coordinate frame ID.
  Q_PROPERTY(QString frameId READ getFrameId WRITE setFrameId NOTIFY rangeChanged)
  /// Message header timestamp.
  Q_PROPERTY(QDateTime timestamp READ getTimestamp WRITE setTimestamp NOTIFY rangeChanged)

public slots:
  float getRange() {
    return publisher_.msgBuffer().range;
  }
  void setRange(float rng) {
    publisher_.msgBuffer().range = rng;
    emit rangeChanged();
  }

  float getFieldOfView() {
    return publisher_.msgBuffer().field_of_view;
  }
  void setFieldOfView(float fov) {
    publisher_.msgBuffer().field_of_view = fov;
    emit rangeChanged();
  }

  float getMinRange() {
    return publisher_.msgBuffer().min_range;
  }
  void setMinRange(float min) {
    publisher_.msgBuffer().min_range = min;
    emit rangeChanged();
  }

  float getMaxRange() {
    return publisher_.msgBuffer().max_range;
  }
  void setMaxRange(float max) {
    publisher_.msgBuffer().max_range = max;
    emit rangeChanged();
  }

  QString getFrameId() {
    return QString::fromStdString(publisher_.msgBuffer().header.frame_id);
  }

  void setFrameId(QString frameId) {
    publisher_.msgBuffer().header.frame_id = frameId.toStdString();
    emit rangeChanged();
  }

  QDateTime getTimestamp() {
    qint64 totalMSecs = static_cast<qint64>(publisher_.msgBuffer().header.stamp.sec) * 1000LL +
                        static_cast<qint64>(publisher_.msgBuffer().header.stamp.nanosec) / 1000000LL;
    return QDateTime::fromMSecsSinceEpoch(totalMSecs);
  }

  void setTimestamp(QDateTime timestamp) {
    publisher_.msgBuffer().header.stamp.sec = timestamp.toSecsSinceEpoch();
    publisher_.msgBuffer().header.stamp.nanosec = (timestamp.toMSecsSinceEpoch() % 1000) * 1000000;
    emit rangeChanged();
  }

signals:
  void rangeChanged();

protected:
  QRosPublisherInterface* interfacePtr(){return &publisher_;}
  QRosTypedPublisher<sensor_msgs::msg::Range> publisher_;
};


/**
 * @brief Subscribes to `sensor_msgs/Range` messages.
 *
 * ### QML usage
 * @code{.qml}
 * QRosRangeSubscriber {
 *     node:  applicationNode
 *     topic: "/altimeter/range"
 *     onRangeChanged: altDisplay.value = range
 * }
 * @endcode
 */
class QRosRangeSubscriber : public QRosSubscriber{
  Q_OBJECT
public:
  /// Range in metres from the last received message.
  Q_PROPERTY(float range READ getRange NOTIFY rangeChanged)
  /// Sensor field of view in radians from the last received message.
  Q_PROPERTY(float fieldOfView READ getFieldOfView NOTIFY rangeChanged)
  /// Minimum valid range from the last received message.
  Q_PROPERTY(float minRange READ getMinRange NOTIFY rangeChanged)
  /// Maximum valid range from the last received message.
  Q_PROPERTY(float maxRange READ getMaxRange NOTIFY rangeChanged)
  /// Frame ID from the last received message header.
  Q_PROPERTY(QString frameId READ getFrameId NOTIFY rangeChanged)
  /// Timestamp from the last received message header.
  Q_PROPERTY(QDateTime timestamp READ getTimestamp NOTIFY rangeChanged)

public slots:
  float getRange() {
    return subscriber_.msgBuffer().range;
  }

  float getFieldOfView() {
    return subscriber_.msgBuffer().field_of_view;
  }

  float getMinRange() {
    return subscriber_.msgBuffer().min_range;
  }

  float getMaxRange() {
    return subscriber_.msgBuffer().max_range;
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
  void rangeChanged();

protected:
  void onMsgReceived() override{
    emit rangeChanged();
  }

private:
  QRosSubscriberInterface* interfacePtr(){return &subscriber_;}
  QRosTypedSubscriber<sensor_msgs::msg::Range> subscriber_;
};

QROS_NS_FOOT
