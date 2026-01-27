#pragma once

#include "qros_object.h"
#include <QDebug>
#include <QTimer>

#include <QElapsedTimer> // Add this for timestamp tracking


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
    rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(queue_size));

    if (latched) {
      qos.transient_local();
      // Optional (often desired for latched/state):
      qos.reliable();
    } else {
      qos.best_effort();          // <-- key change for GUI streams
      qos.durability_volatile();  // explicit
    }

    if (topic.isEmpty()) {
      ros_sub_.reset();
      return;
    }

    try {
      ros_sub_ = ros_node_ptr_->template create_subscription<msg_T>(
          topic.toStdString(),
          qos,
          std::bind(&QRosTypedSubscriber::rosCallback, this, std::placeholders::_1)
          );
    } catch (...) {
      ros_sub_.reset();
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

      // Configure check timer based on new timeout
      configureStaleTimer();
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

    // Record the timestamp of the last message
    last_msg_timestamp_.restart();

    // Clear stale flag if it was set
    if (is_stale_) {
      is_stale_ = false;
      emit isStaleChanged();
    }
  }

  virtual void onMsgReceived() = 0;
  virtual QRosSubscriberInterface* interfacePtr() = 0;

  // Replaced resetStaleTimer with this more reliable approach
  void configureStaleTimer() {
    // Stop existing timer
    if (stale_check_timer_) {
      stale_check_timer_->stop();
    }

    // Only start timer if we have a valid timeout
    if (stale_timeout_ > 0) {
      // Use a shorter interval to check more frequently
      int check_interval = qMin(int(stale_timeout_ * 250), 100); // Check 4 times per timeout period, but max 100ms
      stale_check_timer_->start(check_interval);

      // Initialize or restart the last message timestamp tracker
      if (!last_msg_timestamp_.isValid()) {
        last_msg_timestamp_.start();
      }
    }
  }

  void checkStaleState() {
    // Only check if we have a valid timeout and the timer is running
    if (stale_timeout_ > 0 && last_msg_timestamp_.isValid()) {
      qint64 elapsed = last_msg_timestamp_.elapsed();
      bool should_be_stale = (elapsed > int(stale_timeout_ * 1000));

      // Only update and emit signal if the state changes
      if (should_be_stale != is_stale_) {
        is_stale_ = should_be_stale;
        emit isStaleChanged();
      }
    }
  }

  QString topic_;
  int queue_size_ = 1;
  bool latched_ = false;
  double stale_timeout_ = 0; // in seconds
  bool is_stale_ = false;
  QElapsedTimer last_msg_timestamp_; // To track when the last message was received
  QTimer* stale_check_timer_ = new QTimer(this); // Timer to periodically check stale state

  QRosSubscriber() {
    // Connect the timer to the check function
    connect(stale_check_timer_, &QTimer::timeout, this, &QRosSubscriber::checkStaleState);

    // Start the message timestamp tracker
    last_msg_timestamp_.start();
  }
};

QROS_NS_FOOT
