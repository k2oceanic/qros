#pragma once

#include "qdebug.h"
#include "qros_object.h"

QROS_NS_HEAD

class QRosPublisherInterface{
public:
  typedef std::shared_ptr<QRosPublisherInterface> SharedPtr;
  virtual void setNode(QRosNode* node) = 0;
  virtual void publish() = 0;
  virtual void createRosPub(QString topic, int queue_size) = 0;
  virtual QString getTopic() = 0 ;
};

template <typename msg_T>
class QRosTypedPublisher: public QRosPublisherInterface{
public:
  void setNode(QRosNode* node){
    ros_node_ptr_ = node->getNodePtr();
  }
  void publish(){
    if(ros_pub_)
      ros_pub_->publish(msg_buffer_);
  }
  void createRosPub(QString topic, int queue_size = 10){
    ros_pub_ = ros_node_ptr_->template create_publisher<msg_T>(topic.toStdString(), queue_size);
  }
  QString getTopic(){
    return QString::fromStdString(ros_pub_->get_topic_name());
  }
  msg_T & msgBuffer(){return msg_buffer_;}
  const msg_T getConstBuffer(){return msg_buffer_;}
  msg_T msg_buffer_;
private:
  rclcpp::Node::SharedPtr ros_node_ptr_;
  typename rclcpp::Publisher<msg_T>::SharedPtr ros_pub_;
};

class QRosPublisher : public QRosObject{
  Q_OBJECT
public:
  Q_PROPERTY(QString topic READ getTopic WRITE setTopic NOTIFY topicChanged)
public slots:
  void setTopic(QString topic){
    try{
      topic_ = topic;
      interfacePtr()->setNode(getNode());
      interfacePtr()->createRosPub(topic_,1);
      emit topicChanged();
    }catch(...){
      qWarning() << "invalid topic name: " << topic;
    }
  }
  QString getTopic(){
    return topic_;
  }
  void publish(){
    interfacePtr()->publish();
  }
signals:
  void topicChanged();
protected:
  virtual QRosPublisherInterface * interfacePtr() = 0;
  QString topic_;
};


QROS_NS_FOOT
