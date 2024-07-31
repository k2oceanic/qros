#pragma once

#include "qros_object.h"
#include <QDebug>
#include <QTimer>

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
    if(topic == ""){
      ros_sub_ = nullptr;
      return;
    }
    try {
      ros_sub_ = ros_node_ptr_->template create_subscription<msg_T>(
          topic.toStdString(), qos, std::bind(&QRosTypedSubscriber::rosCallback, this, std::placeholders::_1));
    }
    catch (...) {
      ros_sub_ = nullptr;
      qWarning() << "Failed to create subscriber" << topic;
    }
  }

  QString getTopic() {
    if (ros_sub_)
      return QString::fromStdString(ros_sub_->get_topic_name());
    else
      return QString::fromStdString("");
  }

  msg_T& msgBuffer() { return msg_buffer_; }

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
  Q_PROPERTY(double staleTimeout READ getStaleTimeout WRITE setStaleTimeout NOTIFY staleTimeoutChanged)
  Q_PROPERTY(bool isStale READ isStale NOTIFY isStaleChanged)

public slots:
  void setTopic(QString topic) {
    if (getRosNode() != nullptr) {
      interfacePtr()->setNode(getNode());
      interfacePtr()->setCallback([this]() { handleMsg(); });
      interfacePtr()->subscribe(topic, queue_size_, latched_);
      emit topicChanged();
    } else {
      qWarning() << "Subscriber topic changed before node was set! " << topic;
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

  void setStaleTimeout(double timeout) {
    if (stale_timeout_ != timeout) {
      stale_timeout_ = timeout;
      emit staleTimeoutChanged();
      resetStaleTimer();
    }
  }

  double getStaleTimeout() const {
    return stale_timeout_;
  }

  bool isStale() const {
    return is_stale_;
  }

signals:
  void topicChanged();
  void queueSizeChanged();
  void latchedChanged();
  void staleTimeoutChanged();
  void isStaleChanged();
  void msgReceived();

protected:
  virtual void handleMsg() {
    emit msgReceived();
    onMsgReceived();
    resetStaleTimer();
  }

  virtual void onMsgReceived() = 0;
  virtual QRosSubscriberInterface* interfacePtr() = 0;

  void resetStaleTimer() {
    if (stale_timer_) {
      stale_timer_->stop();
      if(is_stale_){
        is_stale_ = false;
        emit isStaleChanged();
      }
    }
    if (stale_timeout_ > 0) {
      stale_timer_->start(int(stale_timeout_ * 1000));
    }
  }

  QString topic_;
  int queue_size_ = 1;
  bool latched_ = false;
  double stale_timeout_ = 0; // in seconds
  bool is_stale_ = false;
  QTimer* stale_timer_ = new QTimer(this);

  QRosSubscriber() {
    connect(stale_timer_, &QTimer::timeout, this, [this]() {
      is_stale_ = true;
      emit isStaleChanged();
      stale_timer_->stop();
    });
  }
};

QROS_NS_FOOT
