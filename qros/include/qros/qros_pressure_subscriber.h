#pragma once

/**
 * @file qros_pressure_subscriber.h
 * @brief Subscriber for sensor_msgs/msg/FluidPressure.
 */

#include "qros_subscriber.h"
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <QString>

QROS_NS_HEAD

/**
 * @brief Subscribes to `sensor_msgs/FluidPressure` messages.
 *
 * Exposes the fluid pressure reading and its variance as QML properties.
 *
 * ### QML usage
 * @code{.qml}
 * QRosFluidPressureSubscriber {
 *     node:  applicationNode
 *     topic: "/depth_pressure"
 *     onPressureChanged: depthDisplay.pressurePa = pressure
 * }
 * @endcode
 */
class QRosFluidPressureSubscriber : public QRosSubscriber{
  Q_OBJECT
public:
  /// Fluid pressure in Pascals from the last received message.
  Q_PROPERTY(double pressure READ getPressure NOTIFY pressureChanged)
  /// Variance of the pressure reading (0 = unknown).
  Q_PROPERTY(double variance READ getVariance NOTIFY varianceChanged)

public slots:
  double getPressure() {
    return subscriber_.msgBuffer().fluid_pressure;
  }

  double getVariance()  {
    return subscriber_.msgBuffer().variance;
  }

signals:
  void pressureChanged();
  void varianceChanged();

protected:
  void onMsgReceived() override {
    emit pressureChanged();
    emit varianceChanged();
  }

private:
  QRosSubscriberInterface* interfacePtr() { return &subscriber_; }
  QRosTypedSubscriber<sensor_msgs::msg::FluidPressure> subscriber_;
};

QROS_NS_FOOT
