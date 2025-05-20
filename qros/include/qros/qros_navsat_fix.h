#pragma once

#include "qros_subscriber.h"
#include "qros_publisher.h"
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <QString>
#include <QDateTime>

QROS_NS_HEAD

    class QRosNavSatFixPublisher : public QRosPublisher {
  Q_OBJECT
public:
  Q_PROPERTY(double latitude READ getLatitude WRITE setLatitude NOTIFY fixChanged)
  Q_PROPERTY(double longitude READ getLongitude WRITE setLongitude NOTIFY fixChanged)
  Q_PROPERTY(double altitude READ getAltitude WRITE setAltitude NOTIFY fixChanged)
  Q_PROPERTY(int status READ getStatus WRITE setStatus NOTIFY fixChanged)
  Q_PROPERTY(QString frameId READ getFrameId WRITE setFrameId NOTIFY fixChanged)
  Q_PROPERTY(QDateTime timestamp READ getTimestamp WRITE setTimestamp NOTIFY fixChanged)

public slots:
  double getLatitude() {
    return publisher_.msgBuffer().latitude;
  }

  void setLatitude(double lat) {
    publisher_.msgBuffer().latitude = lat;
    emit fixChanged();
  }

  double getLongitude() {
    return publisher_.msgBuffer().longitude;
  }

  void setLongitude(double lon) {
    publisher_.msgBuffer().longitude = lon;
    emit fixChanged();
  }

  double getAltitude() {
    return publisher_.msgBuffer().altitude;
  }

  void setAltitude(double alt) {
    publisher_.msgBuffer().altitude = alt;
    emit fixChanged();
  }

  int getStatus() {
    return publisher_.msgBuffer().status.status;
  }

  void setStatus(int status) {
    publisher_.msgBuffer().status.status = status;
    emit fixChanged();
  }

  QString getFrameId() {
    return QString::fromStdString(publisher_.msgBuffer().header.frame_id);
  }

  void setFrameId(QString frameId) {
    publisher_.msgBuffer().header.frame_id = frameId.toStdString();
    emit fixChanged();
  }

  QDateTime getTimestamp() {
    qint64 totalMSecs = static_cast<qint64>(publisher_.msgBuffer().header.stamp.sec) * 1000LL +
                        static_cast<qint64>(publisher_.msgBuffer().header.stamp.nanosec) / 1000000LL;
    return QDateTime::fromMSecsSinceEpoch(totalMSecs);
  }

  void setTimestamp(QDateTime timestamp) {
    publisher_.msgBuffer().header.stamp.sec = timestamp.toSecsSinceEpoch();
    publisher_.msgBuffer().header.stamp.nanosec = (timestamp.toMSecsSinceEpoch() % 1000) * 1000000;
    emit fixChanged();
  }

signals:
  void fixChanged();

protected:
  QRosPublisherInterface* interfacePtr() override { return &publisher_; }
  QRosTypedPublisher<sensor_msgs::msg::NavSatFix> publisher_;
};


class QRosNavSatFixSubscriber : public QRosSubscriber {
  Q_OBJECT
public:
  Q_PROPERTY(double latitude READ getLatitude NOTIFY fixChanged)
  Q_PROPERTY(double longitude READ getLongitude NOTIFY fixChanged)
  Q_PROPERTY(double altitude READ getAltitude NOTIFY fixChanged)
  Q_PROPERTY(int status READ getStatus NOTIFY fixChanged)
  Q_PROPERTY(QString frameId READ getFrameId NOTIFY fixChanged)
  Q_PROPERTY(QDateTime timestamp READ getTimestamp NOTIFY fixChanged)

public slots:
  double getLatitude() {
    return subscriber_.msgBuffer().latitude;
  }

  double getLongitude() {
    return subscriber_.msgBuffer().longitude;
  }

  double getAltitude() {
    return subscriber_.msgBuffer().altitude;
  }

  int getStatus() {
    return subscriber_.msgBuffer().status.status;
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
  void fixChanged();

protected:
  void onMsgReceived() override {
    emit fixChanged();
  }

private:
  QRosSubscriberInterface* interfacePtr() override { return &subscriber_; }
  QRosTypedSubscriber<sensor_msgs::msg::NavSatFix> subscriber_;
};

QROS_NS_FOOT
