#pragma once

#include "qros_publisher.h"
#include "qros_subscriber.h"
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <QString>

QROS_NS_HEAD

    // Bool Publisher
    class QRosBoolPublisher : public QRosPublisher {
  Q_OBJECT
public:
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

// Bool Subscriber
class QRosBoolSubscriber : public QRosSubscriber {
  Q_OBJECT
public:
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

// Int Publisher
class QRosIntPublisher : public QRosPublisher {
  Q_OBJECT
public:
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

// Int Subscriber
class QRosIntSubscriber : public QRosSubscriber {
  Q_OBJECT
public:
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

// Double Publisher
class QRosDoublePublisher : public QRosPublisher {
  Q_OBJECT
public:
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

// Double Subscriber
class QRosDoubleSubscriber : public QRosSubscriber {
  Q_OBJECT
public:
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
