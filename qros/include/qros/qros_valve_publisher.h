#pragma once

/**
 * @file qros_valve_publisher.h
 * @brief Publisher for hydraulic_interfaces/msg/Valve.
 */

#include "qros_publisher.h"
#include "hydraulic_interfaces/msg/valve.hpp"
#include <QString>

QROS_NS_HEAD

/**
 * @brief Publishes `hydraulic_interfaces/Valve` messages.
 *
 * Sends a single valve command identified by valve ID and set-point.
 *
 * ### QML usage
 * @code{.qml}
 * QRosValvePublisher {
 *     id:      valvePub
 *     node:    applicationNode
 *     topic:   "/hydraulic/valve_cmd"
 *     valveId: 5
 * }
 * Slider {
 *     onValueChanged: { valvePub.setPoint = value; valvePub.publish() }
 * }
 * @endcode
 */
class QRosValvePublisher : public QRosPublisher{
  Q_OBJECT
public:
  /// Hardware valve identifier (1-indexed, board-specific).
  Q_PROPERTY(int valveId READ getValveId WRITE setValveId NOTIFY valveIdChanged)
  /// Commanded position in the 0.0–1.0 range (0 = fully closed, 1 = fully open).
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
  QRosTypedPublisher<hydraulic_interfaces::msg::Valve> publisher_;
};

QROS_NS_FOOT
