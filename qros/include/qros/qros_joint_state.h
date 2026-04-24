#pragma once

/**
 * @file qros_joint_state.h
 * @brief Publisher and subscriber for sensor_msgs/msg/JointState.
 */

#include "qros_subscriber.h"
#include "qros_publisher.h"
#include <sensor_msgs/msg/joint_state.hpp>
#include <QString>
#include <QVector>
#include <QDateTime>

QROS_NS_HEAD

/**
 * @brief Publishes `sensor_msgs/JointState` messages.
 *
 * All parallel arrays (names, positions, velocities, efforts) must be kept
 * the same length by the caller.
 *
 * ### QML usage
 * @code{.qml}
 * QRosJointStatePublisher {
 *     id:         jsPub
 *     node:       applicationNode
 *     topic:      "/joint_commands"
 *     jointNames: ["shoulder", "elbow", "wrist"]
 *     positions:  [shoulderAngle, elbowAngle, wristAngle]
 * }
 * @endcode
 */
class QRosJointStatePublisher : public QRosPublisher {
  Q_OBJECT
public:
  /// Array of joint names (must be the same length as positions/velocities/efforts).
  Q_PROPERTY(QVector<QString> jointNames READ getJointNames WRITE setJointNames NOTIFY jointStateChanged)
  /// Joint position values (rad for revolute, m for prismatic).
  Q_PROPERTY(QVector<double> positions READ getPositions WRITE setPositions NOTIFY jointStateChanged)
  /// Joint velocity values.
  Q_PROPERTY(QVector<double> velocities READ getVelocities WRITE setVelocities NOTIFY jointStateChanged)
  /// Joint effort values (N·m for revolute, N for prismatic).
  Q_PROPERTY(QVector<double> efforts READ getEfforts WRITE setEfforts NOTIFY jointStateChanged)
  /// Message header timestamp.
  Q_PROPERTY(QDateTime timestamp READ getTimestamp WRITE setTimestamp NOTIFY jointStateChanged)

public slots:
  QVector<QString> getJointNames() {
    QVector<QString> names;
    for (const auto& name : publisher_.msgBuffer().name) {
      names.append(QString::fromStdString(name));
    }
    return names;
  }
  void setJointNames(const QVector<QString>& names) {
    publisher_.msgBuffer().name.clear();
    for (const auto& name : names) {
      publisher_.msgBuffer().name.push_back(name.toStdString());
    }
    emit jointStateChanged();
  }

  QVector<double> getPositions() {
    return QVector<double>(publisher_.msgBuffer().position.begin(), publisher_.msgBuffer().position.end());
  }
  void setPositions(const QVector<double>& positions) {
    publisher_.msgBuffer().position = std::vector<double>(positions.begin(), positions.end());
    emit jointStateChanged();
  }

  QVector<double> getVelocities() {
    return QVector<double>(publisher_.msgBuffer().velocity.begin(), publisher_.msgBuffer().velocity.end());
  }
  void setVelocities(const QVector<double>& velocities) {
    publisher_.msgBuffer().velocity = std::vector<double>(velocities.begin(), velocities.end());
    emit jointStateChanged();
  }

  QVector<double> getEfforts() {
    return QVector<double>(publisher_.msgBuffer().effort.begin(), publisher_.msgBuffer().effort.end());
  }
  void setEfforts(const QVector<double>& efforts) {
    publisher_.msgBuffer().effort = std::vector<double>(efforts.begin(), efforts.end());
    emit jointStateChanged();
  }

  QDateTime getTimestamp() {
    qint64 totalMSecs = static_cast<qint64>(publisher_.msgBuffer().header.stamp.sec) * 1000LL +
                        static_cast<qint64>(publisher_.msgBuffer().header.stamp.nanosec) / 1000000LL;
    return QDateTime::fromMSecsSinceEpoch(totalMSecs);
  }

  void setTimestamp(QDateTime timestamp) {
    publisher_.msgBuffer().header.stamp.sec = timestamp.toSecsSinceEpoch();
    publisher_.msgBuffer().header.stamp.nanosec = (timestamp.toMSecsSinceEpoch() % 1000) * 1000000;
    emit jointStateChanged();
  }

signals:
  void jointStateChanged();

protected:
  QRosPublisherInterface* interfacePtr() { return &publisher_; }
  QRosTypedPublisher<sensor_msgs::msg::JointState> publisher_;
};

/**
 * @brief Subscribes to `sensor_msgs/JointState` messages.
 *
 * ### QML usage
 * @code{.qml}
 * QRosJointStateSubscriber {
 *     node:  applicationNode
 *     topic: "/joint_states"
 *     onJointStateChanged: {
 *         for (var i = 0; i < jointNames.length; i++)
 *             console.log(jointNames[i], "pos:", positions[i])
 *     }
 * }
 * @endcode
 */
class QRosJointStateSubscriber : public QRosSubscriber {
  Q_OBJECT
public:
  /// Array of joint names from the last received message.
  Q_PROPERTY(QVector<QString> jointNames READ getJointNames NOTIFY jointStateChanged)
  /// Joint positions from the last received message.
  Q_PROPERTY(QVector<double> positions READ getPositions NOTIFY jointStateChanged)
  /// Joint velocities from the last received message.
  Q_PROPERTY(QVector<double> velocities READ getVelocities NOTIFY jointStateChanged)
  /// Joint efforts from the last received message.
  Q_PROPERTY(QVector<double> efforts READ getEfforts NOTIFY jointStateChanged)
  /// Timestamp from the last received message header.
  Q_PROPERTY(QDateTime timestamp READ getTimestamp NOTIFY jointStateChanged)

public slots:
  QVector<QString> getJointNames() {
    QVector<QString> names;
    for (const auto& name : subscriber_.msgBuffer().name) {
      names.append(QString::fromStdString(name));
    }
    return names;
  }

  QVector<double> getPositions() {
    return QVector<double>(subscriber_.msgBuffer().position.begin(), subscriber_.msgBuffer().position.end());
  }

  QVector<double> getVelocities() {
    return QVector<double>(subscriber_.msgBuffer().velocity.begin(), subscriber_.msgBuffer().velocity.end());
  }

  QVector<double> getEfforts() {
    return QVector<double>(subscriber_.msgBuffer().effort.begin(), subscriber_.msgBuffer().effort.end());
  }

  QDateTime getTimestamp() {
    qint64 totalMSecs = static_cast<qint64>(subscriber_.msgBuffer().header.stamp.sec) * 1000LL +
                        static_cast<qint64>(subscriber_.msgBuffer().header.stamp.nanosec) / 1000000LL;
    return QDateTime::fromMSecsSinceEpoch(totalMSecs);
  }

signals:
  void jointStateChanged();

protected:
  void onMsgReceived() override {
    emit jointStateChanged();
  }

private:
  QRosSubscriberInterface* interfacePtr() { return &subscriber_; }
  QRosTypedSubscriber<sensor_msgs::msg::JointState> subscriber_;
};

QROS_NS_FOOT
