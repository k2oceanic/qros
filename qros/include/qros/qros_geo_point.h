#pragma once

/**
 * @file qros_geo_point.h
 * @brief Publisher and subscriber for geographic_msgs/msg/GeoPoint.
 */

#include "qros_publisher.h"
#include "qros_subscriber.h"
#include <geographic_msgs/msg/geo_point.hpp>
#include <QDateTime>
#include <QObject>
#include <QString>
#include <QVector3D>

QROS_NS_HEAD

/**
 * @brief Publishes `geographic_msgs/GeoPoint` messages.
 *
 * Suitable for publishing geodetic coordinates (WGS-84).
 *
 * ### QML usage
 * @code{.qml}
 * QRosGeoPointPublisher {
 *     node:      applicationNode
 *     topic:     "/home_position"
 *     latitude:  gpsLat
 *     longitude: gpsLon
 *     altitude:  gpsAlt
 * }
 * @endcode
 */
class QRosGeoPointPublisher : public QRosPublisher {
  Q_OBJECT
public:
  /// Latitude in decimal degrees (WGS-84).
  Q_PROPERTY(double latitude READ getLatitude WRITE setLatitude NOTIFY geoPointChanged)
  /// Longitude in decimal degrees (WGS-84).
  Q_PROPERTY(double longitude READ getLongitude WRITE setLongitude NOTIFY geoPointChanged)
  /// Altitude in metres above the WGS-84 ellipsoid.
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

/**
 * @brief Subscribes to `geographic_msgs/GeoPoint` messages.
 *
 * ### QML usage
 * @code{.qml}
 * QRosGeoPointSubscriber {
 *     node:  applicationNode
 *     topic: "/estimated_position"
 *     onGeoPointChanged: map.center = QtPositioning.coordinate(latitude, longitude, altitude)
 * }
 * @endcode
 */
class QRosGeoPointSubscriber : public QRosSubscriber {
  Q_OBJECT
public:
  /// Latitude in decimal degrees from the last received message.
  Q_PROPERTY(double latitude READ getLatitude NOTIFY geoPointChanged)
  /// Longitude in decimal degrees from the last received message.
  Q_PROPERTY(double longitude READ getLongitude NOTIFY geoPointChanged)
  /// Altitude in metres from the last received message.
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
