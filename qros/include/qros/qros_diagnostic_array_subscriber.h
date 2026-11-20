#pragma once

/**
 * @file qros_diagnostic_array_subscriber.h
 * @brief Subscriber for diagnostic_msgs/msg/DiagnosticArray.
 */

#include "qros_subscriber.h"
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <QString>
#include <QVariantList>

QROS_NS_HEAD

/**
 * @brief Subscribes to a `diagnostic_msgs/DiagnosticArray` topic.
 *
 * Exposes the array of status entries as a QML-readable `status` property.
 * Each entry is a QVariantMap with keys:
 *  - `"level"` (int): severity — 0=OK, 1=WARN, 2=ERROR, 3=STALE
 *  - `"name"` (QString): component name
 *  - `"message"` (QString): status description
 *  - `"hardware_id"` (QString): hardware identifier
 *  - `"values"` (QVariantList of QVariantMaps): key/value pairs, each with `"key"` and `"value"`
 *
 * ### QML usage
 * @code{.qml}
 * QRosDiagnosticArraySubscriber {
 *     node:  applicationNode
 *     topic: "/diagnostics"
 *     onStatusChanged: {
 *         for (var i = 0; i < status.length; i++) {
 *             if (status[i].level > 0)
 *                 console.log("FAULT:", status[i].name, status[i].message)
 *         }
 *     }
 * }
 * @endcode
 */
class QRosDiagnosticArraySubscriber : public QRosSubscriber{
  Q_OBJECT
public:
  /// List of DiagnosticStatus entries from the last received array.
  Q_PROPERTY(QVariantList status READ getStatus NOTIFY statusChanged)

public slots:
  /// Returns the current status array as a QVariantList of QVariantMaps.
  QVariantList getStatus() {
    QVariantList statusList;
    for (auto& status : subscriber_.msgBuffer().status) {
      QVariantMap statusMap;
      statusMap["level"] = status.level;
      statusMap["name"] = QString::fromStdString(status.name);
      statusMap["message"] = QString::fromStdString(status.message);
      statusMap["hardware_id"] = QString::fromStdString(status.hardware_id);

      QVariantList valuesList;
      for (const auto& kv : status.values) {
        QVariantMap kvMap;
        kvMap["key"] = QString::fromStdString(kv.key);
        kvMap["value"] = QString::fromStdString(kv.value);
        valuesList.append(kvMap);
      }
      statusMap["values"] = valuesList;

      statusList.append(statusMap);
    }
    return statusList;
  }

signals:
  /// Emitted on every received DiagnosticArray message.
  void statusChanged();

protected:
  void onMsgReceived() override {
    emit statusChanged();
  }

private:
  QRosSubscriberInterface* interfacePtr() { return &subscriber_; }
  QRosTypedSubscriber<diagnostic_msgs::msg::DiagnosticArray> subscriber_;
};

QROS_NS_FOOT
