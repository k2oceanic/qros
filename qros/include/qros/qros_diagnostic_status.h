#pragma once

#include "qros_subscriber.h"
#include "qros_publisher.h"
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <QString>
#include <QVariantMap>

QROS_NS_HEAD

    class QRosDiagnosticStatusSubscriber : public QRosSubscriber {
  Q_OBJECT
public:
  Q_PROPERTY(int level READ getLevel NOTIFY levelChanged)
  Q_PROPERTY(QString name READ getName NOTIFY nameChanged)
  Q_PROPERTY(QString message READ getMessage NOTIFY messageChanged)
  Q_PROPERTY(QString hardwareId READ getHardwareId NOTIFY hardwareIdChanged)
  Q_PROPERTY(QVariantMap values READ getValues NOTIFY valuesChanged)

public slots:
  int getLevel() {
    return subscriber_.msgBuffer().level;
  }

  QString getName() {
    return QString::fromStdString(subscriber_.msgBuffer().name);
  }

  QString getMessage() {
    return QString::fromStdString(subscriber_.msgBuffer().message);
  }

  QString getHardwareId() {
    return QString::fromStdString(subscriber_.msgBuffer().hardware_id);
  }

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

class QRosDiagnosticStatusPublisher : public QRosPublisher {
  Q_OBJECT
public:
  Q_PROPERTY(int level READ getLevel WRITE setLevel NOTIFY levelChanged)
  Q_PROPERTY(QString name READ getName WRITE setName NOTIFY nameChanged)
  Q_PROPERTY(QString message READ getMessage WRITE setMessage NOTIFY messageChanged)
  Q_PROPERTY(QString hardwareId READ getHardwareId WRITE setHardwareId NOTIFY hardwareIdChanged)
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
