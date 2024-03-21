#pragma once

#include "qros_subscriber.h"
#include <std_msgs/msg/string.hpp>
#include <QString>

QROS_NS_HEAD

class QRosStringSubscriber : public QRosSubscriber{
  Q_OBJECT
public:
Q_PROPERTY(QString data READ getData NOTIFY dataChanged)
public slots:
  QString getData() const{return QString::fromStdString(subscriber_.msg_buffer_.data);}

signals:
  void dataChanged();

protected:
  void onMsgReceived() override{
    emit dataChanged();
  }

private:
  QRosSubscriberInterface* interfacePtr(){return &subscriber_;}
  QRosTypedSubscriber<std_msgs::msg::String> subscriber_;
};

QROS_NS_FOOT


