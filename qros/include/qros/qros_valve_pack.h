#pragma once

/**
 * @file qros_valve_pack.h
 * @brief Publisher and subscriber for hydraulic_interfaces/msg/ValvePack.
 */

#include "qros_publisher.h"
#include "qros_subscriber.h"
#include "hydraulic_interfaces/msg/valve_pack.hpp"
#include <QString>
#include <QVector>
#include <QDateTime>

QROS_NS_HEAD

/**
 * @brief Publishes `hydraulic_interfaces/ValvePack` messages.
 *
 * Manages a packed array of valve commands.  Call resizeValves() once to
 * set the array size, then populate individual valves by index or by ID.
 *
 * ### QML usage
 * @code{.qml}
 * QRosValvePackPublisher {
 *     id:    valvePackPub
 *     node:  applicationNode
 *     topic: "/hydraulic/valve_pack_cmd"
 *     Component.onCompleted: resizeValves(8)
 * }
 * Slider {
 *     onValueChanged: {
 *         valvePackPub.setValveById(3, value)
 *         valvePackPub.publish()
 *     }
 * }
 * @endcode
 */
class QRosValvePackPublisher : public QRosPublisher {
  Q_OBJECT
public:
  /// Array of valve IDs for all packed valves.
  Q_PROPERTY(QVector<int> valveIds READ getValveIds WRITE setValveIds NOTIFY valvePackChanged)
  /// Set-points for all packed valves (parallel to valveIds).
  Q_PROPERTY(QVector<double> setPoints READ getSetPoints WRITE setSetPoints NOTIFY valvePackChanged)
  /// Coordinate frame ID.
  Q_PROPERTY(QString frameId READ getFrameId WRITE setFrameId NOTIFY valvePackChanged)
  /// Message header timestamp.
  Q_PROPERTY(QDateTime timestamp READ getTimestamp WRITE setTimestamp NOTIFY valvePackChanged)

public slots:
  /// Resizes the internal valves array to @p size entries (initialised to 0).
  void resizeValves(int size){
    publisher_.msgBuffer().valves.resize(size);
  }

  QVector<int> getValveIds() {
    QVector<int> ids;
    for (const auto& valve : publisher_.msgBuffer().valves) {
      ids.append(valve.valve_id);
    }
    return ids;
  }

  void setValveIds(QVector<int> ids) {
    for (int i = 0; i < ids.size(); ++i) {
      if (i < publisher_.msgBuffer().valves.size()) {
        publisher_.msgBuffer().valves[i].valve_id = ids[i];
      }
    }
    emit valvePackChanged();
  }

  QVector<double> getSetPoints() {
    QVector<double> points;
    for (const auto& valve : publisher_.msgBuffer().valves) {
      points.append(valve.set_point);
    }
    return points;
  }

  void setSetPoints(QVector<double> points) {
    for (int i = 0; i < points.size(); ++i) {
      if (i < publisher_.msgBuffer().valves.size()) {
        publisher_.msgBuffer().valves[i].set_point = points[i];
      }
    }
    emit valvePackChanged();
  }

  /**
   * @brief Sets the valve_id at position @p index by 0-based array index.
   * @param index    0-based position in the valves array.
   * @param valve_id New valve ID.
   */
  void setId(int index, int valve_id) {
    if (index >= 0 && index < publisher_.msgBuffer().valves.size()) {
        publisher_.msgBuffer().valves[index].valve_id = valve_id;
    }
  }

  /**
   * @brief Returns the valve_id at 0-based position @p index.
   * @return -1 if the index is out of range.
   */
  int getId(int index) {
      if (index >= 0 && index < publisher_.msgBuffer().valves.size()) {
          return publisher_.msgBuffer().valves[index].valve_id;
      }
      return -1;
  }

  /**
   * @brief Sets the set-point for the valve with the given ID.
   * @param valve_id  Valve ID to search for.
   * @param value     New set-point.
   * @return True if the valve was found and updated; false otherwise.
   */
  bool setValveById(int valve_id, double value) {
    auto& valves = publisher_.msgBuffer().valves;
    for (size_t i = 0; i < valves.size(); i++) {
      if (valves[i].valve_id == valve_id) {
        valves[i].set_point = value;
        return true;
      }
    }
    return false;
  }

  /**
   * @brief Returns the set-point for the valve with the given ID.
   * @param valve_id  Valve ID to search for.
   * @return Current set-point, or 0.0 if not found.
   */
  double getValveById(int valve_id) {
    auto& valves = publisher_.msgBuffer().valves;
    for (size_t i = 0; i < valves.size(); i++) {
      if (valves[i].valve_id == valve_id) {
        return valves[i].set_point;
      }
    }
    return 0.0;
  }

  QString getFrameId() {
    return QString::fromStdString(publisher_.msgBuffer().header.frame_id);
  }

  void setFrameId(QString frameId) {
    publisher_.msgBuffer().header.frame_id = frameId.toStdString();
    emit valvePackChanged();
  }

  QDateTime getTimestamp() {
    qint64 totalMSecs = static_cast<qint64>(publisher_.msgBuffer().header.stamp.sec) * 1000LL +
                        static_cast<qint64>(publisher_.msgBuffer().header.stamp.nanosec) / 1000000LL;
    return QDateTime::fromMSecsSinceEpoch(totalMSecs);
  }

  void setTimestamp(QDateTime timestamp) {
    publisher_.msgBuffer().header.stamp.sec = timestamp.toSecsSinceEpoch();
    publisher_.msgBuffer().header.stamp.nanosec = (timestamp.toMSecsSinceEpoch() % 1000) * 1000000;
    emit valvePackChanged();
  }

signals:
  void valvePackChanged();

protected:
  QRosPublisherInterface* interfacePtr() { return &publisher_; }
  QRosTypedPublisher<hydraulic_interfaces::msg::ValvePack> publisher_;
};

/**
 * @brief Subscribes to `hydraulic_interfaces/ValvePack` messages.
 *
 * ### QML usage
 * @code{.qml}
 * QRosValvePackSubscriber {
 *     node:  applicationNode
 *     topic: "/hydraulic/valve_pack_feedback"
 *     onValvePackChanged: {
 *         for (var i = 0; i < valveIds.length; i++)
 *             valveDisplays[valveIds[i]].value = setPoints[i]
 *     }
 * }
 * @endcode
 */
class QRosValvePackSubscriber : public QRosSubscriber {
  Q_OBJECT
public:
  /// Valve IDs from the last received message.
  Q_PROPERTY(QVector<int> valveIds READ getValveIds NOTIFY valvePackChanged)
  /// Set-points from the last received message (parallel to valveIds).
  Q_PROPERTY(QVector<double> setPoints READ getSetPoints NOTIFY valvePackChanged)
  /// Frame ID from the last received message header.
  Q_PROPERTY(QString frameId READ getFrameId NOTIFY valvePackChanged)
  /// Timestamp from the last received message header.
  Q_PROPERTY(QDateTime timestamp READ getTimestamp NOTIFY valvePackChanged)

public slots:
  QVector<int> getValveIds() {
    QVector<int> ids;
    for (const auto& valve : subscriber_.msgBuffer().valves) {
      ids.append(valve.valve_id);
    }
    return ids;
  }

  QVector<double> getSetPoints() {
    QVector<double> points;
    for (const auto& valve : subscriber_.msgBuffer().valves) {
      points.append(valve.set_point);
    }
    return points;
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
  void valvePackChanged();

protected:
  void onMsgReceived() override {
    emit valvePackChanged();
  }

  QRosSubscriberInterface* interfacePtr() { return &subscriber_; }
  QRosTypedSubscriber<hydraulic_interfaces::msg::ValvePack> subscriber_;
};

QROS_NS_FOOT
