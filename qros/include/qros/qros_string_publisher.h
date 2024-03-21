#pragma once

#include "qros_publisher.h"
#include <std_msgs/msg/string.hpp>
#include <QString>

QROS_NS_HEAD

class QRosStringPublisher : public QRosPublisher<std_msgs::msg::String>{
  Q_OBJECT
public:
Q_PROPERTY(QString data READ getData WRITE setData NOTIFY dataChanged)

public slots:
  void setTopic(QString topic){setTopicBase(topic);}
  void publish(){
    publishBase();
  }

  QString getData() const{
    return QString::fromStdString(msg_buffer_.data);
  }

  void setData(QString topic){
    msg_buffer_.data = topic.toStdString();
    emit dataChanged();
  }
signals:
  void dataChanged();

protected:
  //QString data;

private:
};

QROS_NS_FOOT


