#pragma once

/**
 * @file qros_diagnostic_status.h
 * @brief Publisher and subscriber for diagnostic_msgs/msg/DiagnosticStatus.
 *
 * @note For aggregated diagnostics driven by QML property bindings, prefer
 * QRosDiagnosticTask + QRosDiagnosticsUpdater over QRosDiagnosticStatusPublisher.
 */

#include "qros_subscriber.h"
#include "qros_publisher.h"
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <QString>
#include <QVariantMap>

QROS_NS_HEAD

/**
 * @brief Subscribes to a `diagnostic_msgs/DiagnosticStatus` topic.
 *
 * Exposes all status fields — level, name, message, hardware_id, and the
 * key/value map — as QML properties updated on every message.
 *
 * ### QML usage
 * @code{.qml}
 * QRosDiagnosticStatusSubscriber {
 *     node:  applicationNode
 *     topic: "/thruster_driver/status"
 *     onLevelChanged: indicator.color = level > 0 ? "red" : "green"
 * }
 * @endcode
 */
class QRosDiagnosticStatusSubscriber : public QRosSubscriber {
  Q_OBJECT
public:
  /// Numeric severity level (0=OK, 1=WARN, 2=ERROR, 3=STALE).
  Q_PROPERTY(int level READ getLevel NOTIFY levelChanged)
  /// Status name string from the message.
  Q_PROPERTY(QString name READ getName NOTIFY nameChanged)
  /// Human-readable status description.
  Q_PROPERTY(QString message READ getMessage NOTIFY messageChanged)
  /// Hardware ID string from the message header.
  Q_PROPERTY(QString hardwareId READ getHardwareId NOTIFY hardwareIdChanged)
  /// Key/value pairs from the message as a QVariantMap.
  Q_PROPERTY(QVariantMap values READ getValues NOTIFY valuesChanged)

public slots:
  /// Returns the last received severity level.
  int getLevel() {
    return subscriber_.msgBuffer().level;
  }

  /// Returns the last received status name.
  QString getName() {
    return QString::fromStdString(subscriber_.msgBuffer().name);
  }

  /// Returns the last received status message.
  QString getMessage() {
    return QString::fromStdString(subscriber_.msgBuffer().message);
  }

  /// Returns the last received hardware ID.
  QString getHardwareId() {
    return QString::fromStdString(subscriber_.msgBuffer().hardware_id);
  }

  /// Returns the last received key/value pairs as a QVariantMap.
  QVariantMap getValues() {
    QVariantMap valuesMap;
    for (const auto& kv : subscriber_.msgBuffer().values) {
      valuesMap[QString::fromStdString(kv.key)] = QString::fromStdString(kv.value);
    }
    return valuesMap;
  }

signals:
  void levelChanged();
  void nameChanged();
  void messageChanged();
  void hardwareIdChanged();
  void valuesChanged();

protected:
  void onMsgReceived() override {
    emit levelChanged();
    emit nameChanged();
    emit messageChanged();
    emit hardwareIdChanged();
    emit valuesChanged();
  }

private:
  QRosSubscriberInterface* interfacePtr() { return &subscriber_; }
  QRosTypedSubscriber<diagnostic_msgs::msg::DiagnosticStatus> subscriber_;
};

/**
 * @brief Publishes a single `diagnostic_msgs/DiagnosticStatus` topic.
 *
 * Exposes all fields as writable properties.  Call publish() after updating
 * them to send a message.
 *
 * @note For automated, timer-driven publishing of multiple health tasks,
 * use QRosDiagnosticTask + QRosDiagnosticsUpdater instead.
 *
 * ### QML usage
 * @code{.qml}
 * QRosDiagnosticStatusPublisher {
 *     id:         statusPub
 *     node:       applicationNode
 *     topic:      "/my_component/status"
 *     hardwareId: "my_component"
 *     name:       "Sensor Health"
 *     level:      sensorOk ? 0 : 2
 *     message:    sensorOk ? "OK" : "Sensor not responding"
 * }
 * Timer { interval: 1000; running: true; onTriggered: statusPub.publish() }
 * @endcode
 */
class QRosDiagnosticStatusPublisher : public QRosPublisher {
  Q_OBJECT
public:
  /// Numeric severity level (0=OK, 1=WARN, 2=ERROR, 3=STALE).
  Q_PROPERTY(int level READ getLevel WRITE setLevel NOTIFY levelChanged)
  /// Status name string.
  Q_PROPERTY(QString name READ getName WRITE setName NOTIFY nameChanged)
  /// Human-readable status description.
  Q_PROPERTY(QString message READ getMessage WRITE setMessage NOTIFY messageChanged)
  /// Hardware ID string.
  Q_PROPERTY(QString hardwareId READ getHardwareId WRITE setHardwareId NOTIFY hardwareIdChanged)
  /// Key/value diagnostic pairs as a QVariantMap.
  Q_PROPERTY(QVariantMap values READ getValues WRITE setValues NOTIFY valuesChanged)

public slots:
  int getLevel() {
    return publisher_.msgBuffer().level;
  }

  void setLevel(int level) {
    publisher_.msgBuffer().level = level;
    emit levelChanged();
  }

  QString getName() {
    return QString::fromStdString(publisher_.msgBuffer().name);
  }

  void setName(const QString& name) {
    publisher_.msgBuffer().name = name.toStdString();
    emit nameChanged();
  }

  QString getMessage() {
    return QString::fromStdString(publisher_.msgBuffer().message);
  }

  void setMessage(const QString& message) {
    publisher_.msgBuffer().message = message.toStdString();
    emit messageChanged();
  }

  QString getHardwareId() {
    return QString::fromStdString(publisher_.msgBuffer().hardware_id);
  }

  void setHardwareId(const QString& hardwareId) {
    publisher_.msgBuffer().hardware_id = hardwareId.toStdString();
    emit hardwareIdChanged();
  }

  QVariantMap getValues() {
    QVariantMap valuesMap;
    for (const auto& kv : publisher_.msgBuffer().values) {
      valuesMap[QString::fromStdString(kv.key)] = QString::fromStdString(kv.value);
    }
    return valuesMap;
  }

  void setValues(const QVariantMap& values) {
    publisher_.msgBuffer().values.clear();
    for (auto it = values.begin(); it != values.end(); ++it) {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = it.key().toStdString();
      kv.value = it.value().toString().toStdString();
      publisher_.msgBuffer().values.push_back(kv);
    }
    emit valuesChanged();
  }

signals:
  void levelChanged();
  void nameChanged();
  void messageChanged();
  void hardwareIdChanged();
  void valuesChanged();

protected:
  QRosPublisherInterface* interfacePtr() { return &publisher_; }
  QRosTypedPublisher<diagnostic_msgs::msg::DiagnosticStatus> publisher_;
};

QROS_NS_FOOT
