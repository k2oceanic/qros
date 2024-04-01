#pragma once

#include "qros_subscriber.h"
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <QString>

QROS_NS_HEAD

class QRosFluidPressureSubscriber : public QRosSubscriber{
  Q_OBJECT
public:
  Q_PROPERTY(double pressure READ getPressure NOTIFY pressureChanged)
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
