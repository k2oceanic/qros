#pragma once

/**
 * @file qros_float32.h
 * @brief Publishers and subscribers for float32 and float32 multi-array messages.
 *
 * Provides QML types for:
 *  - std_msgs/Float32           → QRosFloat32Publisher / QRosFloat32Subscriber
 *  - std_msgs/Float32MultiArray → QRosFloat32MultiArrayPublisher / QRosFloat32MultiArraySubscriber
 */

#include "qros_publisher.h"
#include "qros_subscriber.h"
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <QString>
#include <QVector>

QROS_NS_HEAD

/**
 * @brief Publishes `std_msgs/Float32` messages.
 *
 * ### QML usage
 * @code{.qml}
 * QRosFloat32Publisher { node: applicationNode; topic: "/set_gain"; data: gainSlider.value }
 * @endcode
 */
class QRosFloat32Publisher : public QRosPublisher{
  Q_OBJECT
public:
  /// The single-precision float value to publish.
  Q_PROPERTY(float data READ getData WRITE setData NOTIFY dataChanged)
public slots:
  float getData(){
    return publisher_.msgBuffer().data;
  }
  void setData(float data){
    publisher_.msgBuffer().data = data;
    emit dataChanged();
  }
signals:
  void dataChanged();

protected:
  QRosPublisherInterface * interfacePtr(){return &publisher_;}
  QRosTypedPublisher<std_msgs::msg::Float32> publisher_;
};

/**
 * @brief Subscribes to `std_msgs/Float32` messages.
 *
 * ### QML usage
 * @code{.qml}
 * QRosFloat32Subscriber {
 *     node: applicationNode; topic: "/temperature_c"
 *     onDataChanged: tempDisplay.value = data
 * }
 * @endcode
 */
class QRosFloat32Subscriber : public QRosSubscriber{
  Q_OBJECT
public:
  /// The last received single-precision float value.
  Q_PROPERTY(float data READ getData NOTIFY dataChanged)
public slots:
  float getData() {
    return subscriber_.msgBuffer().data;
  }

signals:
  void dataChanged();

protected:
  void onMsgReceived() override{
    emit dataChanged();
  }

private:
  QRosSubscriberInterface* interfacePtr(){return &subscriber_;}
  QRosTypedSubscriber<std_msgs::msg::Float32> subscriber_;
};

/**
 * @brief Publishes `std_msgs/Float32MultiArray` messages.
 *
 * ### QML usage
 * @code{.qml}
 * QRosFloat32MultiArrayPublisher {
 *     id: arrayPub; node: applicationNode; topic: "/coefficients"
 * }
 * // Set values from JS: arrayPub.data = [1.0, 0.5, 0.25]
 * @endcode
 */
class QRosFloat32MultiArrayPublisher : public QRosPublisher {
  Q_OBJECT
public:
  /// Array of single-precision float values to publish.
  Q_PROPERTY(QVector<float> data READ getData WRITE setData NOTIFY dataChanged)
public slots:
  QVector<float> getData() {
    QVector<float> qData;
    for (const auto &value : publisher_.msgBuffer().data) {
      qData.append(value);
    }
    return qData;
  }
  void setData(const QVector<float> &data) {
    publisher_.msgBuffer().data.clear();
    for (const auto &value : data) {
      publisher_.msgBuffer().data.push_back(value);
    }
    emit dataChanged();
  }
signals:
  void dataChanged();

protected:
  QRosPublisherInterface* interfacePtr() { return &publisher_; }
  QRosTypedPublisher<std_msgs::msg::Float32MultiArray> publisher_;
};

/**
 * @brief Subscribes to `std_msgs/Float32MultiArray` messages.
 *
 * ### QML usage
 * @code{.qml}
 * QRosFloat32MultiArraySubscriber {
 *     node: applicationNode; topic: "/filter_coeffs"
 *     onDataChanged: for (var i = 0; i < data.length; i++) model.set(i, {value: data[i]})
 * }
 * @endcode
 */
class QRosFloat32MultiArraySubscriber : public QRosSubscriber {
  Q_OBJECT
public:
  /// The last received array of single-precision float values.
  Q_PROPERTY(QVector<float> data READ getData NOTIFY dataChanged)
public slots:
  QVector<float> getData() {
    QVector<float> qData;
    for (const auto &value : subscriber_.msgBuffer().data) {
      qData.append(value);
    }
    return qData;
  }

signals:
  void dataChanged();

protected:
  void onMsgReceived() override {
    emit dataChanged();
  }

private:
  QRosSubscriberInterface* interfacePtr() { return &subscriber_; }
  QRosTypedSubscriber<std_msgs::msg::Float32MultiArray> subscriber_;
};

QROS_NS_FOOT
