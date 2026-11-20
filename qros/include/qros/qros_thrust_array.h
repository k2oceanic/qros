#pragma once

/**
 * @file qros_thrust_array.h
 * @brief Publisher and subscriber for propulsion_interfaces/msg/ThrustArray.
 */

#include "qros_publisher.h"
#include "qros_subscriber.h"
#include <propulsion_interfaces/msg/thrust_array.hpp>
#include <QString>
#include <QVector>
#include <QDateTime>

QROS_NS_HEAD

/**
 * @brief Publishes `propulsion_interfaces/ThrustArray` messages.
 *
 * Commands multiple thrusters simultaneously.  Call resizeThrusts() once to
 * set the array size, then populate scales and proportional values.
 *
 * ### QML usage
 * @code{.qml}
 * QRosThrustArrayPublisher {
 *     id:   thrustPub
 *     node: applicationNode
 *     topic: "/thruster_array/cmd"
 *     Component.onCompleted: resizeThrusts(6)
 * }
 * // From a joystick handler:
 * // thrustPub.proportionalValues = [surge, sway, heave, roll, pitch, yaw]
 * // thrustPub.publish()
 * @endcode
 */
class QRosThrustArrayPublisher : public QRosPublisher {
  Q_OBJECT
public:
  /// Scale values (max thrust in Newtons) for each thruster.
  Q_PROPERTY(QVector<float> scales READ getScales WRITE setScales NOTIFY thrustArrayChanged)
  /// Proportional thrust commands (−1.0 to 1.0) for each thruster.
  Q_PROPERTY(QVector<float> proportionalValues READ getProportionalValues WRITE setProportionalValues NOTIFY thrustArrayChanged)
  /// Coordinate frame ID.
  Q_PROPERTY(QString frameId READ getFrameId WRITE setFrameId NOTIFY thrustArrayChanged)
  /// Message header timestamp.
  Q_PROPERTY(QDateTime timestamp READ getTimestamp WRITE setTimestamp NOTIFY thrustArrayChanged)

public slots:
  /// Resizes the thrusters array to @p size entries (initialised to 0).
  void resizeThrusts(int size) {
    publisher_.msgBuffer().thrusts.resize(size);
  }

  QVector<float> getScales() {
    QVector<float> scales;
    for (const auto& thrust : publisher_.msgBuffer().thrusts) {
      scales.append(thrust.scale);
    }
    return scales;
  }

  void setScales(QVector<float> scales) {
    for (int i = 0; i < scales.size(); ++i) {
      if (i < static_cast<int>(publisher_.msgBuffer().thrusts.size())) {
        publisher_.msgBuffer().thrusts[i].scale = scales[i];
      }
    }
    emit thrustArrayChanged();
  }

  QVector<float> getProportionalValues() {
    QVector<float> values;
    for (const auto& thrust : publisher_.msgBuffer().thrusts) {
      values.append(thrust.proportional_value);
    }
    return values;
  }

  void setProportionalValues(QVector<float> values) {
    for (int i = 0; i < values.size(); ++i) {
      if (i < static_cast<int>(publisher_.msgBuffer().thrusts.size())) {
        publisher_.msgBuffer().thrusts[i].proportional_value = values[i];
      }
    }
    emit thrustArrayChanged();
  }

  QString getFrameId() {
    return QString::fromStdString(publisher_.msgBuffer().header.frame_id);
  }

  void setFrameId(QString frameId) {
    publisher_.msgBuffer().header.frame_id = frameId.toStdString();
    emit thrustArrayChanged();
  }

  QDateTime getTimestamp() {
    qint64 totalMSecs = static_cast<qint64>(publisher_.msgBuffer().header.stamp.sec) * 1000LL +
                        static_cast<qint64>(publisher_.msgBuffer().header.stamp.nanosec) / 1000000LL;
    return QDateTime::fromMSecsSinceEpoch(totalMSecs);
  }

  void setTimestamp(QDateTime timestamp) {
    publisher_.msgBuffer().header.stamp.sec = timestamp.toSecsSinceEpoch();
    publisher_.msgBuffer().header.stamp.nanosec = (timestamp.toMSecsSinceEpoch() % 1000) * 1000000;
    emit thrustArrayChanged();
  }

signals:
  void thrustArrayChanged();

protected:
  QRosPublisherInterface* interfacePtr() { return &publisher_; }
  QRosTypedPublisher<propulsion_interfaces::msg::ThrustArray> publisher_;
};


/**
 * @brief Subscribes to `propulsion_interfaces/ThrustArray` messages.
 *
 * ### QML usage
 * @code{.qml}
 * QRosThrustArraySubscriber {
 *     node:  applicationNode
 *     topic: "/thruster_array/feedback"
 *     onThrustArrayChanged: {
 *         for (var i = 0; i < proportionalValues.length; i++)
 *             thrusterBars[i].value = proportionalValues[i]
 *     }
 * }
 * @endcode
 */
class QRosThrustArraySubscriber : public QRosSubscriber {
  Q_OBJECT
public:
  /// Scale values from the last received message.
  Q_PROPERTY(QVector<float> scales READ getScales NOTIFY thrustArrayChanged)
  /// Proportional values from the last received message.
  Q_PROPERTY(QVector<float> proportionalValues READ getProportionalValues NOTIFY thrustArrayChanged)
  /// Frame ID from the last received message header.
  Q_PROPERTY(QString frameId READ getFrameId NOTIFY thrustArrayChanged)
  /// Timestamp from the last received message header.
  Q_PROPERTY(QDateTime timestamp READ getTimestamp NOTIFY thrustArrayChanged)

public slots:
  QVector<float> getScales() {
    QVector<float> scales;
    for (const auto& thrust : subscriber_.msgBuffer().thrusts) {
      scales.append(thrust.scale);
    }
    return scales;
  }

  QVector<float> getProportionalValues() {
    QVector<float> values;
    for (const auto& thrust : subscriber_.msgBuffer().thrusts) {
      values.append(thrust.proportional_value);
    }
    return values;
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
  void thrustArrayChanged();

protected:
  void onMsgReceived() override {
    emit thrustArrayChanged();
  }

private:
  QRosSubscriberInterface* interfacePtr() { return &subscriber_; }
  QRosTypedSubscriber<propulsion_interfaces::msg::ThrustArray> subscriber_;
};

QROS_NS_FOOT
