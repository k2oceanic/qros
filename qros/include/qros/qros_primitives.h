#pragma once

/**
 * @file qros_primitives.h
 * @brief Publishers and subscribers for standard ROS 2 primitive message types.
 *
 * Provides QML types for:
 *  - std_msgs/Bool     → QRosBoolPublisher / QRosBoolSubscriber
 *  - std_msgs/Int32    → QRosIntPublisher / QRosIntSubscriber
 *  - std_msgs/Int64    → QRosInt64Publisher / QRosInt64Subscriber
 *  - std_msgs/Float64  → QRosDoublePublisher / QRosDoubleSubscriber
 */

#include "qros_publisher.h"
#include "qros_subscriber.h"
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/float64.hpp>
#include <QString>

QROS_NS_HEAD

/**
 * @brief Publishes `std_msgs/Bool` messages.
 *
 * ### QML usage
 * @code{.qml}
 * QRosBoolPublisher { node: applicationNode; topic: "/arm_system"; data: armButton.checked }
 * @endcode
 */
class QRosBoolPublisher : public QRosPublisher {
  Q_OBJECT
public:
  /// The boolean value to publish.
  Q_PROPERTY(bool data READ getData WRITE setData NOTIFY dataChanged)
public slots:
  bool getData() {
    return publisher_.msgBuffer().data;
  }
  void setData(bool data) {
    publisher_.msgBuffer().data = data;
    emit dataChanged();
  }
signals:
  void dataChanged();

protected:
  QRosPublisherInterface* interfacePtr() { return &publisher_; }
  QRosTypedPublisher<std_msgs::msg::Bool> publisher_;
};

/**
 * @brief Subscribes to `std_msgs/Bool` messages.
 *
 * ### QML usage
 * @code{.qml}
 * QRosBoolSubscriber {
 *     node: applicationNode; topic: "/system_armed"
 *     onDataChanged: armedIndicator.active = data
 * }
 * @endcode
 */
class QRosBoolSubscriber : public QRosSubscriber {
  Q_OBJECT
public:
  /// The last received boolean value.
  Q_PROPERTY(bool data READ getData NOTIFY dataChanged)
public slots:
  bool getData() {
    return subscriber_.msgBuffer().data;
  }
signals:
  void dataChanged();

protected:
  void onMsgReceived() override {
    emit dataChanged();
  }

private:
  QRosSubscriberInterface* interfacePtr() { return &subscriber_; }
  QRosTypedSubscriber<std_msgs::msg::Bool> subscriber_;
};

/**
 * @brief Publishes `std_msgs/Int32` messages.
 *
 * ### QML usage
 * @code{.qml}
 * QRosIntPublisher { node: applicationNode; topic: "/mode"; data: modeSelector.currentIndex }
 * @endcode
 */
class QRosIntPublisher : public QRosPublisher {
  Q_OBJECT
public:
  /// The integer value to publish.
  Q_PROPERTY(int data READ getData WRITE setData NOTIFY dataChanged)
public slots:
  int getData() {
    return publisher_.msgBuffer().data;
  }
  void setData(int data) {
    publisher_.msgBuffer().data = data;
    emit dataChanged();
  }
signals:
  void dataChanged();

protected:
  QRosPublisherInterface* interfacePtr() { return &publisher_; }
  QRosTypedPublisher<std_msgs::msg::Int32> publisher_;
};

/**
 * @brief Subscribes to `std_msgs/Int32` messages.
 *
 * ### QML usage
 * @code{.qml}
 * QRosIntSubscriber {
 *     node: applicationNode; topic: "/active_mode"
 *     onDataChanged: modeLabel.text = data
 * }
 * @endcode
 */
class QRosIntSubscriber : public QRosSubscriber {
  Q_OBJECT
public:
  /// The last received integer value.
  Q_PROPERTY(int data READ getData NOTIFY dataChanged)
public slots:
  int getData() {
    return subscriber_.msgBuffer().data;
  }
signals:
  void dataChanged();

protected:
  void onMsgReceived() override {
    emit dataChanged();
  }

private:
  QRosSubscriberInterface* interfacePtr() { return &subscriber_; }
  QRosTypedSubscriber<std_msgs::msg::Int32> subscriber_;
};

/**
 * @brief Publishes `std_msgs/Int64` messages.
 */
class QRosInt64Publisher : public QRosPublisher {
  Q_OBJECT
public:
  /// The 64-bit integer value to publish.
  Q_PROPERTY(qint64 data READ getData WRITE setData NOTIFY dataChanged)
public slots:
  qint64 getData() {
    return publisher_.msgBuffer().data;
  }
  void setData(qint64 data) {
    publisher_.msgBuffer().data = data;
    emit dataChanged();
  }
signals:
  void dataChanged();

protected:
  QRosPublisherInterface* interfacePtr() override { return &publisher_; }
  QRosTypedPublisher<std_msgs::msg::Int64> publisher_;
};

/**
 * @brief Subscribes to `std_msgs/Int64` messages.
 */
class QRosInt64Subscriber : public QRosSubscriber {
  Q_OBJECT
public:
  /// The last received 64-bit integer value.
  Q_PROPERTY(qint64 data READ getData NOTIFY dataChanged)
public slots:
  qint64 getData() {
    return subscriber_.msgBuffer().data;
  }
signals:
  void dataChanged();

protected:
  void onMsgReceived() override {
    emit dataChanged();
  }

private:
  QRosSubscriberInterface* interfacePtr() override { return &subscriber_; }
  QRosTypedSubscriber<std_msgs::msg::Int64> subscriber_;
};


/**
 * @brief Publishes `std_msgs/Float64` messages.
 *
 * ### QML usage
 * @code{.qml}
 * QRosDoublePublisher {
 *     id: depthPub; node: applicationNode; topic: "/commanded_depth"
 * }
 * Slider { onValueChanged: { depthPub.data = value; depthPub.publish() } }
 * @endcode
 */
class QRosDoublePublisher : public QRosPublisher {
  Q_OBJECT
public:
  /// The double-precision value to publish.
  Q_PROPERTY(double data READ getData WRITE setData NOTIFY dataChanged)
public slots:
  double getData() {
    return publisher_.msgBuffer().data;
  }
  void setData(double data) {
    publisher_.msgBuffer().data = data;
    emit dataChanged();
  }
signals:
  void dataChanged();

protected:
  QRosPublisherInterface* interfacePtr() { return &publisher_; }
  QRosTypedPublisher<std_msgs::msg::Float64> publisher_;
};

/**
 * @brief Subscribes to `std_msgs/Float64` messages.
 *
 * ### QML usage
 * @code{.qml}
 * QRosDoubleSubscriber {
 *     node: applicationNode; topic: "/depth"
 *     onDataChanged: depthGauge.value = data
 * }
 * @endcode
 */
class QRosDoubleSubscriber : public QRosSubscriber {
  Q_OBJECT
public:
  /// The last received double-precision value.
  Q_PROPERTY(double data READ getData NOTIFY dataChanged)
public slots:
  double getData() {
    return subscriber_.msgBuffer().data;
  }
signals:
  void dataChanged();

protected:
  void onMsgReceived() override {
    emit dataChanged();
  }

private:
  QRosSubscriberInterface* interfacePtr() { return &subscriber_; }
  QRosTypedSubscriber<std_msgs::msg::Float64> subscriber_;
};

QROS_NS_FOOT
