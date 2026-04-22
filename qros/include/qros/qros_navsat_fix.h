#pragma once

/**
 * @file qros_navsat_fix.h
 * @brief Publisher and subscriber for sensor_msgs/msg/NavSatFix.
 */

#include "qros_subscriber.h"
#include "qros_publisher.h"
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <QString>
#include <QDateTime>
#include <QVariantList>

QROS_NS_HEAD

/**
 * @brief Publishes `sensor_msgs/NavSatFix` messages.
 *
 * Exposes all NavSatFix fields — latitude, longitude, altitude, GNSS status,
 * service mask, 3×3 position covariance, and covariance type — as writable
 * QML properties.
 *
 * ### QML usage
 * @code{.qml}
 * QRosNavSatFixPublisher {
 *     node:      applicationNode
 *     topic:     "/gps/fix_override"
 *     latitude:  gpsLat
 *     longitude: gpsLon
 *     altitude:  gpsAlt
 *     status:    1  // STATUS_FIX
 * }
 * @endcode
 */
class QRosNavSatFixPublisher : public QRosPublisher {
  Q_OBJECT
public:
  /// Latitude in decimal degrees (WGS-84).
  Q_PROPERTY(double latitude READ getLatitude WRITE setLatitude NOTIFY fixChanged)
  /// Longitude in decimal degrees (WGS-84).
  Q_PROPERTY(double longitude READ getLongitude WRITE setLongitude NOTIFY fixChanged)
  /// Altitude in metres above the WGS-84 ellipsoid.
  Q_PROPERTY(double altitude READ getAltitude WRITE setAltitude NOTIFY fixChanged)
  /// GNSS fix status (-1=NO_FIX, 0=FIX, 1=SBAS_FIX, 2=GBAS_FIX).
  Q_PROPERTY(int status READ getStatus WRITE setStatus NOTIFY fixChanged)
  /// GNSS service bitmask (GPS=1, GLONASS=2, COMPASS=4, GALILEO=8).
  Q_PROPERTY(int service READ getService WRITE setService NOTIFY fixChanged)
  /// 3×3 position covariance matrix as a flat 9-element list (row-major, m²).
  Q_PROPERTY(QVariantList covariance READ getCovariance WRITE setCovariance NOTIFY fixChanged)
  /// Covariance type (0=UNKNOWN, 1=APPROXIMATED, 2=DIAGONAL_KNOWN, 3=KNOWN).
  Q_PROPERTY(int covarianceType READ getCovarianceType WRITE setCovarianceType NOTIFY fixChanged)
  /// Coordinate frame ID.
  Q_PROPERTY(QString frameId READ getFrameId WRITE setFrameId NOTIFY fixChanged)
  /// Message header timestamp.
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

  int getService() {
    return publisher_.msgBuffer().status.service;
  }
  void setService(int service) {
    publisher_.msgBuffer().status.service = service;
    emit fixChanged();
  }

  QVariantList getCovariance() {
    QVariantList list;
    const auto& cov = publisher_.msgBuffer().position_covariance;
    for (size_t i = 0; i < 9; ++i) {
      list.append(cov[i]);
    }
    return list;
  }
  void setCovariance(QVariantList cov) {
    auto& msgCov = publisher_.msgBuffer().position_covariance;
    for (size_t i = 0; i < 9 && i < static_cast<size_t>(cov.size()); ++i) {
      msgCov[i] = cov[i].toDouble();
    }
    emit fixChanged();
  }

  int getCovarianceType() {
    return publisher_.msgBuffer().position_covariance_type;
  }
  void setCovarianceType(int type) {
    publisher_.msgBuffer().position_covariance_type = type;
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

/**
 * @brief Subscribes to `sensor_msgs/NavSatFix` messages.
 *
 * ### QML usage
 * @code{.qml}
 * QRosNavSatFixSubscriber {
 *     node:  applicationNode
 *     topic: "/gps/fix"
 *     onFixChanged: map.center = QtPositioning.coordinate(latitude, longitude, altitude)
 * }
 * @endcode
 */
class QRosNavSatFixSubscriber : public QRosSubscriber {
  Q_OBJECT
public:
  /// Latitude in decimal degrees from the last received fix.
  Q_PROPERTY(double latitude READ getLatitude NOTIFY fixChanged)
  /// Longitude in decimal degrees from the last received fix.
  Q_PROPERTY(double longitude READ getLongitude NOTIFY fixChanged)
  /// Altitude in metres from the last received fix.
  Q_PROPERTY(double altitude READ getAltitude NOTIFY fixChanged)
  /// GNSS fix status from the last received fix.
  Q_PROPERTY(int status READ getStatus NOTIFY fixChanged)
  /// GNSS service bitmask from the last received fix.
  Q_PROPERTY(int service READ getService NOTIFY fixChanged)
  /// Position covariance matrix (9-element flat list) from the last received fix.
  Q_PROPERTY(QVariantList covariance READ getCovariance NOTIFY fixChanged)
  /// Covariance type from the last received fix.
  Q_PROPERTY(int covarianceType READ getCovarianceType NOTIFY fixChanged)
  /// Frame ID from the last received message header.
  Q_PROPERTY(QString frameId READ getFrameId NOTIFY fixChanged)
  /// Timestamp from the last received message header.
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

  int getService() {
    return subscriber_.msgBuffer().status.service;
  }

  QVariantList getCovariance() {
    QVariantList list;
    const auto& cov = subscriber_.msgBuffer().position_covariance;
    for (size_t i = 0; i < 9; ++i) {
      list.append(cov[i]);
    }
    return list;
  }

  int getCovarianceType() {
    return subscriber_.msgBuffer().position_covariance_type;
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
