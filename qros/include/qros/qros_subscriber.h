#pragma once

#include "qros_object.h"

QROS_NS_HEAD

template <typename msg_T>
class QRosSubscriber : public QRosObject{

public:

protected:
    msg_T last_msg_;
    void subscribeBase(QString topic);
    virtual void msgReceivedBase() = 0;

private:
    void rosCallback(const typename msg_T::SharedPtr msg);
    typename rclcpp::Subscription<msg_T>::SharedPtr ros_sub_;

};

template<typename msg_T>
void QRosSubscriber<msg_T>::subscribeBase(QString topic){
    ros_sub_ = getRosNode()->template create_subscription<msg_T>(
        topic.toStdString(), 1, std::bind(&QRosSubscriber::rosCallback, this, std::placeholders::_1));
}

template<typename msg_T>
void QRosSubscriber<msg_T>::rosCallback(const typename msg_T::SharedPtr msg){
    last_msg_ = *msg;
    msgReceivedBase();
}

QROS_NS_FOOT
