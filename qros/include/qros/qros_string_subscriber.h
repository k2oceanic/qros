#pragma once

#include "qros_subscriber.h"
#include <std_msgs/msg/string.hpp>
#include <QString>

QROS_NS_HEAD

class QRosString : public QRosTypedMessage <std_msgs::msg::String>{
  Q_OBJECT
public:
  Q_PROPERTY(QString data READ getData NOTIFY dataChanged)
public slots:
  QString getData() {return QString::fromStdString(raw().data);}
signals:
  void dataChanged();
};

class QRosStringSubscriber : public QRosSubscriber{
  Q_OBJECT
public:

private:
  QRosSubscriberInterface* interfacePtr(){return &subscriber_;}
  QRosTypedSubscriber<std_msgs::msg::String> subscriber_;
};

QROS_NS_FOOT


