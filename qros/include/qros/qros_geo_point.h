#pragma once

#include "qros_publisher.h"
#include "qros_subscriber.h"
#include <geographic_msgs/msg/geo_point.hpp>
#include <QDateTime>
#include <QObject>
#include <QString>
#include <QVector3D>

QROS_NS_HEAD

    class QRosGeoPointPublisher : public QRosPublisher {
  Q_OBJECT
public:
  Q_PROPERTY(double latitude READ getLatitude WRITE setLatitude NOTIFY geoPointChanged)
  Q_PROPERTY(double longitude READ getLongitude WRITE setLongitude NOTIFY geoPointChanged)
  Q_PROPERTY(double altitude READ getAltitude WRITE setAltitude NOTIFY geoPointChanged)

public slots:
  double getLatitude()  {
    return publisher_.msgBuffer().latitude;
  }

  void setLatitude(double lat) {
    publisher_.msgBuffer().latitude = lat;
    emit geoPointChanged();
  }

  double getLongitude()  {
    return publisher_.msgBuffer().longitude;
  }

  void setLongitude(double lon) {
    publisher_.msgBuffer().longitude = lon;
    emit geoPointChanged();
  }

  double getAltitude()  {
    return publisher_.msgBuffer().altitude;
  }

  void setAltitude(double alt) {
    publisher_.msgBuffer().altitude = alt;
    emit geoPointChanged();
  }

signals:
  void geoPointChanged();

protected:
  QRosPublisherInterface* interfacePtr() override { return &publisher_; }
  QRosTypedPublisher<geographic_msgs::msg::GeoPoint> publisher_;
};

class QRosGeoPointSubscriber : public QRosSubscriber {
  Q_OBJECT
public:
  Q_PROPERTY(double latitude READ getLatitude NOTIFY geoPointChanged)
  Q_PROPERTY(double longitude READ getLongitude NOTIFY geoPointChanged)
  Q_PROPERTY(double altitude READ getAltitude NOTIFY geoPointChanged)

public slots:
  double getLatitude() {
    return subscriber_.msgBuffer().latitude;
  }

  double getLongitude() {
    return subscriber_.msgBuffer().longitude;
  }

  double getAltitude()  {
    return subscriber_.msgBuffer().altitude;
  }

signals:
  void geoPointChanged();

protected:
  void onMsgReceived() override {
    emit geoPointChanged();
  }

private:
  QRosSubscriberInterface* interfacePtr() override { return &subscriber_; }
  QRosTypedSubscriber<geographic_msgs::msg::GeoPoint> subscriber_;
};

QROS_NS_FOOT
