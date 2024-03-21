#pragma once

#include "qros_object.h"

QROS_NS_HEAD

template <typename msg_T>
class QRosPublisher : public QRosObject{

public:

protected:
    msg_T msg_buffer_;
    void setTopicBase(QString topic);
    virtual void publishBase(){
      ros_pub_->publish(msg_buffer_);
    }

private:
    typename rclcpp::Publisher<msg_T>::SharedPtr ros_pub_;
};

template<typename msg_T>
void QRosPublisher<msg_T>::setTopicBase(QString topic){
  ros_pub_ = getRosNode()->template create_publisher<msg_T>(topic.toStdString(), 10);
}


QROS_NS_FOOT
