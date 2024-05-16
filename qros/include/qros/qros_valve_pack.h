#pragma once

#include "qros_publisher.h"
#include "qros_subscriber.h"
#include "hydraulic_interfaces/msg/valve_pack.hpp" // Include the correct message type
#include <QString>
#include <QVector>
#include <QDateTime>

QROS_NS_HEAD

class QRosValvePackPublisher : public QRosPublisher {
  Q_OBJECT
public:
  Q_PROPERTY(QVector<int> valveIds READ getValveIds WRITE setValveIds NOTIFY valvePackChanged)
  Q_PROPERTY(QVector<double> setPoints READ getSetPoints WRITE setSetPoints NOTIFY valvePackChanged)
  Q_PROPERTY(QString frameId READ getFrameId WRITE setFrameId NOTIFY valvePackChanged)
  Q_PROPERTY(QDateTime timestamp READ getTimestamp WRITE setTimestamp NOTIFY valvePackChanged)

public slots:
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

class QRosValvePackSubscriber : public QRosSubscriber {
  Q_OBJECT
public:
  Q_PROPERTY(QVector<int> valveIds READ getValveIds NOTIFY valvePackChanged)
  Q_PROPERTY(QVector<double> setPoints READ getSetPoints NOTIFY valvePackChanged)
  Q_PROPERTY(QString frameId READ getFrameId NOTIFY valvePackChanged)
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
