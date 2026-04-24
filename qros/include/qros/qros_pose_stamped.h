#pragma once

/**
 * @file qros_pose_stamped.h
 * @brief Publisher and subscriber for geometry_msgs/msg/PoseStamped.
 */

#include "qros_subscriber.h"
#include "qros_publisher.h"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <QString>
#include <QQuaternion>
#include <QVector3D>
#include <QDateTime>
#include <QTime>

QROS_NS_HEAD

/**
 * @brief Publishes `geometry_msgs/PoseStamped` messages.
 *
 * ### QML usage
 * @code{.qml}
 * QRosPoseStampedPublisher {
 *     node:        applicationNode
 *     topic:       "/goal_pose"
 *     position:    Qt.vector3d(x, y, z)
 *     orientation: Qt.quaternion(w, x, y, z)
 *     frameId:     "map"
 * }
 * @endcode
 */
class QRosPoseStampedPublisher : public QRosPublisher{
  Q_OBJECT
public:
  /// Position (x, y, z) in metres.
  Q_PROPERTY(QVector3D position READ getPosition WRITE setPosition NOTIFY poseChanged)
  /// Orientation quaternion (w, x, y, z).
  Q_PROPERTY(QQuaternion orientation READ getOrientation WRITE setOrientation NOTIFY poseChanged)
  /// Coordinate frame ID (e.g. "map", "odom").
  Q_PROPERTY(QString frameId READ getFrameId WRITE setFrameId NOTIFY poseChanged)
  /// Message header timestamp.
  Q_PROPERTY(QDateTime timestamp READ getTimestamp WRITE setTimestamp NOTIFY poseChanged)

public slots:
  QVector3D getPosition(){
    auto position = publisher_.msgBuffer().pose.position;
    return QVector3D(position.x, position.y, position.z);
  }
  void setPosition(QVector3D pos){
    publisher_.msgBuffer().pose.position.x = pos.x();
    publisher_.msgBuffer().pose.position.y = pos.y();
    publisher_.msgBuffer().pose.position.z = pos.z();
    emit poseChanged();
  }

  QQuaternion getOrientation(){
    auto orientation = publisher_.msgBuffer().pose.orientation;
    return QQuaternion(orientation.w, orientation.x, orientation.y, orientation.z);
  }
  void setOrientation(QQuaternion ori){
    publisher_.msgBuffer().pose.orientation.x = ori.x();
    publisher_.msgBuffer().pose.orientation.y = ori.y();
    publisher_.msgBuffer().pose.orientation.z = ori.z();
    publisher_.msgBuffer().pose.orientation.w = ori.scalar();
    emit poseChanged();
  }

  QString getFrameId() {
    return QString::fromStdString(publisher_.msgBuffer().header.frame_id);
  }

  void setFrameId(QString frameId) {
    publisher_.msgBuffer().header.frame_id = frameId.toStdString();
    emit poseChanged();
  }

  QDateTime getTimestamp() {
    qint64 totalMSecs = static_cast<qint64>(publisher_.msgBuffer().header.stamp.sec) * 1000LL +
                        static_cast<qint64>(publisher_.msgBuffer().header.stamp.nanosec) / 1000000LL;
    return QDateTime::fromMSecsSinceEpoch(totalMSecs);
  }

  void setTimestamp(QDateTime timestamp) {
    publisher_.msgBuffer().header.stamp.sec = timestamp.toSecsSinceEpoch();
    publisher_.msgBuffer().header.stamp.nanosec = (timestamp.toMSecsSinceEpoch() % 1000) * 1000000;
    emit poseChanged();
  }

signals:
  void poseChanged();

protected:
  QRosPublisherInterface* interfacePtr(){return &publisher_;}
  QRosTypedPublisher<geometry_msgs::msg::PoseStamped> publisher_;
};



/**
 * @brief Subscribes to `geometry_msgs/PoseStamped` messages.
 *
 * ### QML usage
 * @code{.qml}
 * QRosPoseStampedSubscriber {
 *     node:  applicationNode
 *     topic: "/robot_pose"
 *     onPoseChanged: {
 *         robotMarker.x = position.x
 *         robotMarker.y = position.y
 *     }
 * }
 * @endcode
 */
class QRosPoseStampedSubscriber : public QRosSubscriber{
  Q_OBJECT
public:
  /// Position from the last received message (x, y, z in metres).
  Q_PROPERTY(QVector3D position READ getPosition NOTIFY poseChanged)
  /// Orientation quaternion from the last received message.
  Q_PROPERTY(QQuaternion orientation READ getOrientation NOTIFY poseChanged)
  /// Frame ID from the last received message header.
  Q_PROPERTY(QString frameId READ getFrameId NOTIFY poseChanged)
  /// Timestamp from the last received message header.
  Q_PROPERTY(QDateTime timestamp READ getTimestamp NOTIFY poseChanged)

public slots:
  QVector3D getPosition() {
    auto position = subscriber_.msgBuffer().pose.position;
    return QVector3D(position.x, position.y, position.z);
  }

  QQuaternion getOrientation() {
    auto orientation = subscriber_.msgBuffer().pose.orientation;
    return QQuaternion(orientation.w, orientation.x, orientation.y, orientation.z);
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
  void poseChanged();

protected:
  void onMsgReceived() override{
    emit poseChanged();
  }

private:
  QRosSubscriberInterface* interfacePtr(){return &subscriber_;}
  QRosTypedSubscriber<geometry_msgs::msg::PoseStamped> subscriber_;
};

QROS_NS_FOOT
