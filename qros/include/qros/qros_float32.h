#pragma once

#include "qros_publisher.h"
#include "qros_subscriber.h"
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <QString>
#include <QVector>

QROS_NS_HEAD

class QRosFloat32Publisher : public QRosPublisher{
  Q_OBJECT
public:
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

class QRosFloat32Subscriber : public QRosSubscriber{
  Q_OBJECT
public:
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

class QRosFloat32MultiArrayPublisher : public QRosPublisher {
  Q_OBJECT
public:
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

class QRosFloat32MultiArraySubscriber : public QRosSubscriber {
  Q_OBJECT
public:
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
