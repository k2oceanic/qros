#pragma once

/**
 * @file qros_temperature_subscriber.h
 * @brief Subscriber for sensor_msgs/msg/Temperature.
 */

#include "qros_subscriber.h"
#include <sensor_msgs/msg/temperature.hpp>
#include <QString>

QROS_NS_HEAD

/**
 * @brief Subscribes to `sensor_msgs/Temperature` messages.
 *
 * Exposes the temperature reading and its variance as QML properties.
 *
 * ### QML usage
 * @code{.qml}
 * QRosTemperatureSubscriber {
 *     node:  applicationNode
 *     topic: "/motor_temp"
 *     onTemperatureChanged: tempGauge.value = temperature
 * }
 * @endcode
 */
class QRosTemperatureSubscriber : public QRosSubscriber{
  Q_OBJECT
public:
  /// Temperature in degrees Celsius from the last received message.
  Q_PROPERTY(double temperature READ getTemperature NOTIFY temperatureChanged)
  /// Variance of the temperature reading (0 = unknown).
  Q_PROPERTY(double variance READ getVariance NOTIFY varianceChanged)

public slots:
  double getTemperature() {
    return subscriber_.msgBuffer().temperature;
  }

  double getVariance() {
    return subscriber_.msgBuffer().variance;
  }

signals:
  void temperatureChanged();
  void varianceChanged();

protected:
  void onMsgReceived() override {
    emit temperatureChanged();
    emit varianceChanged();
  }

private:
  QRosSubscriberInterface* interfacePtr() { return &subscriber_; }
  QRosTypedSubscriber<sensor_msgs::msg::Temperature> subscriber_;
};

QROS_NS_FOOT
