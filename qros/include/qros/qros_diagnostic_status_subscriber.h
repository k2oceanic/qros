#pragma once

#include "qros_subscriber.h"
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <QString>
#include <QVariantMap>

QROS_NS_HEAD

class QRosDiagnosticStatusSubscriber : public QRosSubscriber{
  Q_OBJECT
public:
  Q_PROPERTY(quint8 level READ getLevel NOTIFY levelChanged)
  Q_PROPERTY(QString name READ getName NOTIFY nameChanged)
  Q_PROPERTY(QString message READ getMessage NOTIFY messageChanged)
  Q_PROPERTY(QString hardwareId READ getHardwareId NOTIFY hardwareIdChanged)
  Q_PROPERTY(QVariantMap values READ getValues NOTIFY valuesChanged)

public slots:
  quint8 getLevel()  {
    return subscriber_.msgBuffer().level;
  }

  QString getName()  {
    return QString::fromStdString(subscriber_.msgBuffer().name);
  }

  QString getMessage()  {
    return QString::fromStdString(subscriber_.msgBuffer().message);
  }

  QString getHardwareId()  {
    return QString::fromStdString(subscriber_.msgBuffer().hardware_id);
  }

  QVariantMap getValues()  {
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

QROS_NS_FOOT
