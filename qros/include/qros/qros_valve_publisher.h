#pragma once

#include "qros_publisher.h"
#include "hydraulic_interfaces/msg/valve.hpp" 
#include <QString>

QROS_NS_HEAD

class QRosValvePublisher : public QRosPublisher{
  Q_OBJECT
public:
  Q_PROPERTY(int valveId READ getValveId WRITE setValveId NOTIFY valveIdChanged)
  Q_PROPERTY(double proportionalValue READ getProportionalValue WRITE setProportionalValue NOTIFY proportionalValueChanged)

public slots:
  int getValveId() const {
    return publisher_.msg_buffer_.valve_id;
  }
  void setValveId(int id) {
    publisher_.msg_buffer_.valve_id = id;
    emit valveIdChanged();
  }

  double getProportionalValue() const {
    return publisher_.msg_buffer_.proportional_value;
  }
  void setProportionalValue(double value) {
    publisher_.msg_buffer_.proportional_value = value;
    emit proportionalValueChanged();
  }

signals:
  void valveIdChanged();
  void proportionalValueChanged();

protected:
  QRosPublisherInterface * interfacePtr() { return &publisher_; }
  QRosTypedPublisher<hydraulic_interfaces::msg::Valve> publisher_; // Use your custom message type
};

QROS_NS_FOOT
