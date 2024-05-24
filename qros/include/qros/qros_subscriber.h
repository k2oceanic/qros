#pragma once

#include "qros_object.h"
#include <QDebug>

QROS_NS_HEAD


class QRosSubscriberInterface {
public:
  typedef std::shared_ptr<QRosSubscriberInterface> SharedPtr;
  virtual void setNode(QRosNode* node) = 0;
  virtual void subscribe(QString topic, int queue_size, bool latched) = 0;
  virtual QString getTopic() = 0;
  virtual void setCallback(std::function<void()> callback) = 0;
};

template <typename msg_T>
class QRosTypedSubscriber: public QRosSubscriberInterface {
public:
  void setNode(QRosNode* node) {
    ros_node_ptr_ = node->getNodePtr();
  }

  void subscribe(QString topic, int queue_size = 1, bool latched = false) {
    auto qos = latched ? rclcpp::QoS(rclcpp::KeepLast(queue_size)).transient_local() : rclcpp::QoS(queue_size);
    ros_sub_ = ros_node_ptr_->template create_subscription<msg_T>(
        topic.toStdString(), qos, std::bind(&QRosTypedSubscriber::rosCallback, this, std::placeholders::_1));
  }

  QString getTopic() {
    return QString::fromStdString(ros_sub_->get_topic_name());
  }

  msg_T & msgBuffer() { return msg_buffer_; }

  void setCallback(std::function<void()> callback) {
    callback_ = callback;
  }

private:
  msg_T msg_buffer_;
  void rosCallback(const typename msg_T::SharedPtr msg) {
    msg_buffer_ = *msg;
    callback_();
  }

  rclcpp::Node::SharedPtr ros_node_ptr_;
  typename rclcpp::Subscription<msg_T>::SharedPtr ros_sub_;
  std::function<void()> callback_;
};

class QRosSubscriber : public QRosObject {
  Q_OBJECT
public:
  Q_PROPERTY(QString topic READ getTopic WRITE setTopic NOTIFY topicChanged)
  Q_PROPERTY(int queueSize READ getQueueSize WRITE setQueueSize NOTIFY queueSizeChanged)
  Q_PROPERTY(bool latched READ isLatched WRITE setLatched NOTIFY latchedChanged)

public slots:
  void setTopic(QString topic) {
    if (getRosNode() != nullptr) {
      interfacePtr()->setNode(getNode());
      interfacePtr()->setCallback([this]() { handleMsg(); });
      interfacePtr()->subscribe(topic, queue_size_, latched_);
      emit topicChanged();
    } else {
      qWarning() << "subscriber topic changed before node was set! " << topic;
    }
  }

  QString getTopic() {
    return interfacePtr()->getTopic();
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

signals:
  void topicChanged();
  void queueSizeChanged();
  void latchedChanged();
  void msgReceived();

protected:
  virtual void handleMsg() {
    emit msgReceived();
    onMsgReceived();
  }

  virtual void onMsgReceived() = 0;
  virtual QRosSubscriberInterface * interfacePtr() = 0;

  QString topic_;
  int queue_size_ = 10;
  bool latched_ = false;
};


QROS_NS_FOOT
