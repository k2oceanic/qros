#pragma once

#include "qros_subscriber.h"
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <QString>
#include <QVariantList>

QROS_NS_HEAD

class QRosDiagnosticArraySubscriber : public QRosSubscriber{
  Q_OBJECT
public:
  Q_PROPERTY(QVariantList status READ getStatus NOTIFY statusChanged)

public slots:
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
