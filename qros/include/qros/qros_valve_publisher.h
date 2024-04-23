#pragma once

#include "qros_publisher.h"
#include "hydraulic_interfaces/msg/valve.hpp" 
#include <QString>

QROS_NS_HEAD

class QRosValvePublisher : public QRosPublisher{
  Q_OBJECT
public:
  Q_PROPERTY(int valveId READ getValveId WRITE setValveId NOTIFY valveIdChanged)
  Q_PROPERTY(double setPoint READ getSetPoint WRITE setSetPoint NOTIFY setPointChanged)

public slots:
  int getValveId() {
    return publisher_.msgBuffer().valve_id;
  }
  void setValveId(int id) {
    publisher_.msgBuffer().valve_id = id;
    emit valveIdChanged();
  }

  double getSetPoint(){
    return publisher_.msgBuffer().set_point;
  }
  void setSetPoint(double value) {
    publisher_.msgBuffer().set_point = value;
    emit setPointChanged();
  }

signals:
  void valveIdChanged();
  void setPointChanged();

protected:
  QRosPublisherInterface * interfacePtr() { return &publisher_; }
  QRosTypedPublisher<hydraulic_interfaces::msg::Valve> publisher_; // Use your custom message type
};

QROS_NS_FOOT
