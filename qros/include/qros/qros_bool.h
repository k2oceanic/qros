#pragma once

#include "qros_publisher.h"
#include "qros_subscriber.h"
#include <std_msgs/msg/bool.hpp>
#include <QString>

QROS_NS_HEAD

class QRosBoolPublisher : public QRosPublisher{
  Q_OBJECT
public:
Q_PROPERTY(bool data READ getData WRITE setData NOTIFY dataChanged)
public slots:
  bool getData(){
    return publisher_.msgBuffer().data;
  }
  void setData(bool data){
    publisher_.msgBuffer().data = data;
    emit dataChanged();
  }
signals:
  void dataChanged();

protected:
  QRosPublisherInterface * interfacePtr(){return &publisher_;}
  QRosTypedPublisher<std_msgs::msg::Bool> publisher_;
};


class QRosBoolSubscriber : public QRosSubscriber{
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
  void onMsgReceived() override{
    emit dataChanged();
  }

private:
  QRosSubscriberInterface* interfacePtr(){return &subscriber_;}
  QRosTypedSubscriber<std_msgs::msg::Bool> subscriber_;
};



QROS_NS_FOOT


