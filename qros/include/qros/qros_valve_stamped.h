#pragma once

/**
 * @file qros_valve_stamped.h
 * @brief Publisher for hydraulic_interfaces/msg/ValveStamped.
 */

#include "qros_publisher.h"
#include "hydraulic_interfaces/msg/valve_stamped.hpp"
#include <QString>
#include <QDateTime>

QROS_NS_HEAD

/**
 * @brief Publishes `hydraulic_interfaces/ValveStamped` messages.
 *
 * Like QRosValvePublisher but includes a header with frame ID and timestamp.
 * Use this when valve command history or frame-relative commands are needed.
 *
 * ### QML usage
 * @code{.qml}
 * QRosValveStampedPublisher {
 *     node:     applicationNode
 *     topic:    "/hydraulic/valve_stamped_cmd"
 *     valveId:  2
 *     setPoint: controlSlider.value
 *     frameId:  "hydraulic_manifold"
 * }
 * @endcode
 */
class QRosValveStampedPublisher : public QRosPublisher{
  Q_OBJECT
public:
  /// Hardware valve identifier.
  Q_PROPERTY(int valveId READ getValveId WRITE setValveId NOTIFY valveStateChanged)
  /// Commanded position (0.0–1.0).
  Q_PROPERTY(double setPoint READ getSetPoint WRITE setSetPoint NOTIFY valveStateChanged)
  /// Coordinate frame ID.
  Q_PROPERTY(QString frameId READ getFrameId WRITE setFrameId NOTIFY valveStateChanged)
  /// Message header timestamp.
  Q_PROPERTY(QDateTime timestamp READ getTimestamp WRITE setTimestamp NOTIFY valveStateChanged)

public slots:
  int getValveId() {
    return publisher_.msgBuffer().valve.valve_id;
  }
  void setValveId(int id) {
    publisher_.msgBuffer().valve.valve_id = id;
    emit valveStateChanged();
  }

  double getSetPoint() {
    return publisher_.msgBuffer().valve.set_point;
  }
  void setSetPoint(double value) {
    publisher_.msgBuffer().valve.set_point = value;
    emit valveStateChanged();
  }

  QString getFrameId() {
    return QString::fromStdString(publisher_.msgBuffer().header.frame_id);
  }
  void setFrameId(QString frameId) {
    publisher_.msgBuffer().header.frame_id = frameId.toStdString();
    emit valveStateChanged();
  }

  QDateTime getTimestamp() {
    qint64 totalMSecs = static_cast<qint64>(publisher_.msgBuffer().header.stamp.sec) * 1000LL +
                        static_cast<qint64>(publisher_.msgBuffer().header.stamp.nanosec) / 1000000LL;
    return QDateTime::fromMSecsSinceEpoch(totalMSecs);
  }
  void setTimestamp(QDateTime timestamp) {
    publisher_.msgBuffer().header.stamp.sec = timestamp.toSecsSinceEpoch();
    publisher_.msgBuffer().header.stamp.nanosec = (timestamp.toMSecsSinceEpoch() % 1000) * 1000000;
    emit valveStateChanged();
  }

signals:
  void valveStateChanged();

protected:
  QRosPublisherInterface * interfacePtr() { return &publisher_; }
  QRosTypedPublisher<hydraulic_interfaces::msg::ValveStamped> publisher_;
};

QROS_NS_FOOT
