#pragma once

#include "qros_publisher.h"
#include <std_msgs/msg/string.hpp>
#include <QString>

QROS_NS_HEAD



class QRosStringPublisher : public QRosPublisher{
  Q_OBJECT
public:
Q_PROPERTY(QString data READ getData WRITE setData NOTIFY dataChanged)
public slots:
  QString getData(){
    return QString::fromStdString(publisher_.msgBuffer().data);
  }
  void setData(QString topic){
    publisher_.msgBuffer().data = topic.toStdString();
    emit dataChanged();
  }
signals:
  void dataChanged();

protected:
  QRosPublisherInterface * interfacePtr(){return &publisher_;}
  QRosTypedPublisher<std_msgs::msg::String> publisher_;
};

QROS_NS_FOOT


