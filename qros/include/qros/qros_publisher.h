#pragma once

#include "qdebug.h"
#include "qros_object.h"

QROS_NS_HEAD

class QRosPublisherInterface{
public:
  typedef std::shared_ptr<QRosPublisherInterface> SharedPtr;
  virtual void setNode(QRosNode* node) = 0;
  virtual void publish() = 0;
  virtual void createRosPub(QString topic, int queue_size, bool latched) = 0;
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
  void createRosPub(QString topic, int queue_size = 10, bool latched = false) {
    auto qos = latched ? rclcpp::QoS(rclcpp::KeepLast(queue_size)).transient_local() : rclcpp::QoS(queue_size);
    ros_pub_ = ros_node_ptr_->template create_publisher<msg_T>(topic.toStdString(), qos);
  }
  QString getTopic(){
    return QString::fromStdString(ros_pub_->get_topic_name());
  }
  msg_T & msgBuffer(){return msg_buffer_;}
  const msg_T getConstBuffer(){return msg_buffer_;}
private:
  msg_T msg_buffer_;
  rclcpp::Node::SharedPtr ros_node_ptr_;
  typename rclcpp::Publisher<msg_T>::SharedPtr ros_pub_;
};

class QRosPublisher : public QRosObject{
  Q_OBJECT
public:
  Q_PROPERTY(QString topic READ getTopic WRITE setTopic NOTIFY topicChanged)
  Q_PROPERTY(int queueSize READ getQueueSize WRITE setQueueSize NOTIFY queueSizeChanged)
  Q_PROPERTY(bool latched READ isLatched WRITE setLatched NOTIFY latchedChanged)

public slots:
  void setTopic(QString topic){
    try{
      topic_ = topic;
      interfacePtr()->setNode(getNode());
      interfacePtr()->createRosPub(topic_, queue_size_, latched_); 
      emit topicChanged();
    }catch(...){
      qWarning() << "invalid topic name: " << topic;
    }
  }
  QString getTopic() const {
    return topic_;
  }

  void setQueueSize(int queueSize) {
    if (queue_size_ != queueSize) {
      queue_size_ = queueSize;
      emit queueSizeChanged();
    }
  }
  int getQueueSize() const {
    return queue_size_;
  }

  void setLatched(bool latched) {
    if (latched_ != latched) {
      latched_ = latched;
      emit latchedChanged();
    }
  }
  bool isLatched() const {
    return latched_;
  }

  void publish(){
    interfacePtr()->publish();
  }

signals:
  void topicChanged();
  void queueSizeChanged();
  void latchedChanged();

protected:
  virtual QRosPublisherInterface * interfacePtr() = 0;
  QString topic_;
  int queue_size_ = 10;
  bool latched_ = false;
};

QROS_NS_FOOT
