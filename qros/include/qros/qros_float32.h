#pragma once

#include "qros_publisher.h"
#include "qros_subscriber.h"
#include <std_msgs/msg/float32.hpp>
#include <QString>

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

QROS_NS_FOOT
