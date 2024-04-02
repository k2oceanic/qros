#pragma once

#include "qros_subscriber.h"
#include <std_msgs/msg/string.hpp>
#include <QString>

QROS_NS_HEAD

class QRosMessage : public QObject {
  Q_OBJECT
public:
  explicit QRosMessage(QObject *parent = nullptr) : QObject(parent) {}
public slots:
  virtual QString messageType() const = 0;
signals:
  void dataChanged();
};

template<typename msg_T>
class QRosMessageType : public QRosMessage {
public:
  explicit QRosMessageType(msg_T * ros_msg){
    setRosMessagePtr(ros_msg);
  }
  virtual QString messageType() const override {
    return QString::fromStdString(rosidl_generator_traits::data_type<msg_T>());
  }
  msg_T & rosMessage(){return *ros_msg_;}
  virtual void setRosMessagePtr(msg_T * ros_msg){
    ros_msg_ = ros_msg;
    emit dataChanged();
    onDataChaged();
  }
protected:
  virtual void onDataChaged(){}
  msg_T * ros_msg_;
};

QROS_NS_FOOT


