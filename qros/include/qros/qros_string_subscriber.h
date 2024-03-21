#pragma once

#include "qros_subscriber.h"
#include <std_msgs/msg/string.hpp>
#include <QString>

QROS_NS_HEAD

    class QRosStringSubscriber : public QRosSubscriber<std_msgs::msg::String>{
  Q_OBJECT
public:
Q_PROPERTY(QString data READ getData NOTIFY dataChanged)

public slots:
  void subscribe(QString topic){subscribeBase(topic);}
  QString getData() const{return data;}

signals:
  void msgReceived();
  void dataChanged();

protected:
  void msgReceivedBase() override{
    data = QString::fromStdString(last_msg_.data);
    emit dataChanged();
    emit msgReceived();}

  QString data;

private:

    };

    QROS_NS_FOOT


