#pragma once

#include "qros_object.h"
#include <QDebug>

QROS_NS_HEAD


class QRosSubscriberInterface{
public:
  typedef std::shared_ptr<QRosSubscriberInterface> SharedPtr;
  virtual void setNode(QRosNode* node) = 0;
  // virtual void publish() = 0;
  virtual void subscribe(QString topic, int queue_size) = 0;
  virtual QString getTopic() = 0 ;
  virtual void setCallback(std::function<void()> callback) = 0;
};

template <typename msg_T>
class QRosTypedSubscriber: public QRosSubscriberInterface{
public:
  void setNode(QRosNode* node){
    ros_node_ptr_ = node->getNodePtr();
  }
  // void publish(){
  //   ros_pub_->publish(msg_buffer_);
  // }
  void subscribe(QString topic, int queue_size = 1){
    if(topic == ""){
      ros_sub_ = nullptr;
      return;
    }
    try{
    ros_sub_ = ros_node_ptr_->template create_subscription<msg_T>(
        topic.toStdString(), queue_size, std::bind(&QRosTypedSubscriber::rosCallback, this, std::placeholders::_1));
    }
    catch (...) {
      ros_sub_ = nullptr;
      qWarning() << "Failed to create subscriber" <<topic;
    }
  }
  QString getTopic(){
    if(ros_sub_)
      return QString::fromStdString(ros_sub_->get_topic_name());
    else
      return QString::fromStdString("");
  }
  msg_T & msgBuffer(){return msg_buffer_;}
  void setCallback(std::function<void()> callback){
    callback_=callback;
  }
private:
  msg_T msg_buffer_;
  void rosCallback(const typename msg_T::SharedPtr msg);
  rclcpp::Node::SharedPtr ros_node_ptr_;
  typename rclcpp::Subscription<msg_T>::SharedPtr ros_sub_;
  std::function<void()> callback_;
};

template<typename msg_T>
void QRosTypedSubscriber<msg_T>::rosCallback(const typename msg_T::SharedPtr msg){
  msg_buffer_ = *msg;
  callback_();
}

class QRosSubscriber : public QRosObject{
  Q_OBJECT
public:
  Q_PROPERTY(QString topic READ getTopic WRITE setTopic NOTIFY topicChanged)
public slots:
  void setTopic(QString topic){
    if(getRosNode()!=nullptr){
      interfacePtr()->setNode(getNode());
      interfacePtr()->setCallback([this]() { handleMsg();});
      interfacePtr()->subscribe(topic,1);
      emit topicChanged();
    }else{
      qWarning() << "subcriber topic changed before node was set! " <<topic;
    }
  }
  QString getTopic(){
    return interfacePtr()->getTopic();
  }
signals:
  void topicChanged();
  void msgReceived();
protected:
  virtual void handleMsg(){
    emit msgReceived();
    onMsgReceived();
  }
  virtual void onMsgReceived() = 0;
  virtual QRosSubscriberInterface * interfacePtr() = 0;
};

QROS_NS_FOOT
