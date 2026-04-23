#pragma once

/**
 * @file qros_subscriber.h
 * @brief Subscriber base classes for the qros library.
 *
 * Provides two layers:
 *  - QRosTypedSubscriber<msg_T>  — type-erased C++ template that owns the
 *    rclcpp subscription and message buffer.
 *  - QRosSubscriber              — QObject base exposed to QML; subclasses
 *    return a QRosTypedSubscriber via interfacePtr() and override
 *    onMsgReceived() to emit type-specific signals.
 */

#include "qros_object.h"
#include <QDebug>
#include <QTimer>
#include <QElapsedTimer>

QROS_NS_HEAD

/**
 * @brief Abstract interface for all typed subscribers.
 *
 * Provides a uniform, type-erased API so QRosSubscriber can drive any
 * message type without knowing it at compile time.
 */
class QRosSubscriberInterface {
public:
    typedef std::shared_ptr<QRosSubscriberInterface> SharedPtr;

    /// Binds the subscriber to the rclcpp node held by @p node.
    virtual void setNode(QRosNode* node) = 0;

    /**
     * @brief Creates the rclcpp subscription.
     * @param topic      ROS topic name.
     * @param queue_size QoS history depth.
     * @param latched    If true, uses reliable + transient_local QoS.
     */
    virtual void subscribe(QString topic, int queue_size, bool latched) = 0;

    /// Returns the fully-resolved topic name.
    virtual QString getTopic() = 0;

    /**
     * @brief Registers the callback invoked on every incoming message.
     * @param callback  Zero-argument callable; typically a lambda that
     *                  calls handleMsg() on the owning QRosSubscriber.
     */
    virtual void setCallback(std::function<void()> callback) = 0;
};

/**
 * @brief Concrete, type-specific subscriber template.
 *
 * Owns the rclcpp::Subscription<msg_T> and a single message buffer.
 * The callback set via setCallback() is invoked on every received message
 * from the ROS executor thread; it should only update state and emit Qt signals
 * (Qt will marshal the signal to the main thread automatically).
 *
 * ### QoS policy
 * - `latched = false` (default): best_effort + durability_volatile — low overhead,
 *   suitable for high-frequency sensor streams.
 * - `latched = true`: reliable + transient_local — ensures late subscribers
 *   receive the last published value, suitable for configuration topics.
 *
 * @tparam msg_T  ROS 2 message type (e.g. sensor_msgs::msg::Imu).
 */
template <typename msg_T>
class QRosTypedSubscriber : public QRosSubscriberInterface {
public:
    /// @copydoc QRosSubscriberInterface::setNode
    void setNode(QRosNode* node) {
        ros_node_ptr_ = node->getNodePtr();
    }

    /// @copydoc QRosSubscriberInterface::subscribe
    void subscribe(QString topic, int queue_size = 1, bool latched = false) {
        rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(queue_size));
        if (latched) {
            qos.transient_local();
            qos.reliable();
        } else {
            qos.best_effort();
            qos.durability_volatile();
        }
        if (topic.isEmpty()) { ros_sub_.reset(); return; }
        try {
            ros_sub_ = ros_node_ptr_->template create_subscription<msg_T>(
                topic.toStdString(), qos,
                std::bind(&QRosTypedSubscriber::rosCallback, this, std::placeholders::_1));
        } catch (...) {
            ros_sub_.reset();
            qWarning() << "Failed to create subscriber" << topic;
        }
    }

    /// Returns the fully-resolved topic name, or empty string if not subscribed.
    QString getTopic() {
        return ros_sub_ ? QString::fromStdString(ros_sub_->get_topic_name()) : QString();
    }

    /**
     * @brief Returns a mutable reference to the last received message.
     *
     * Valid after at least one message has been received.
     */
    msg_T& msgBuffer() { return msg_buffer_; }

    /// @copydoc QRosSubscriberInterface::setCallback
    void setCallback(std::function<void()> callback) { callback_ = callback; }

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

/**
 * @brief QML-exposed base class for all qros subscribers.
 *
 * Concrete subclasses (e.g. QRosDoubleSubscriber) implement interfacePtr()
 * to return their QRosTypedSubscriber<msg_T>, and override onMsgReceived()
 * to emit type-specific signals when a message arrives.
 *
 * ### Stale detection
 * Set `staleTimeout` (seconds) to enable automatic staleness tracking.
 * If no message arrives within that window, `isStale` becomes `true`.
 * Set to 0 (default) to disable the check entirely.
 *
 * ### QML usage
 * @code{.qml}
 * QRosDoubleSubscriber {
 *     node:         applicationNode
 *     topic:        "/pressure"
 *     staleTimeout: 2.0       // optional: flag stale after 2 s
 *     onMsgReceived: gauge.value = data
 *     onIsStaleChanged: gauge.color = isStale ? "grey" : "green"
 * }
 * @endcode
 */
class QRosSubscriber : public QRosObject {
    Q_OBJECT
public:
    /// ROS topic to subscribe to.  Setting this creates the rclcpp subscription.
    Q_PROPERTY(QString topic        READ getTopic        WRITE setTopic        NOTIFY topicChanged)
    /// QoS history depth.
    Q_PROPERTY(int     queueSize    READ getQueueSize    WRITE setQueueSize    NOTIFY queueSizeChanged)
    /// If true, uses reliable + transient_local QoS (latched topic).
    Q_PROPERTY(bool    latched      READ isLatched       WRITE setLatched      NOTIFY latchedChanged)
    /**
     * @brief Staleness timeout in seconds (0 = disabled).
     *
     * When non-zero, a background timer checks whether a message has been
     * received within the timeout window.  If not, isStale is set to true.
     */
    Q_PROPERTY(double  staleTimeout READ getStaleTimeout WRITE setStaleTimeout NOTIFY staleTimeoutChanged)
    /// True if no message has been received within staleTimeout seconds.
    Q_PROPERTY(bool    isStale      READ isStale                               NOTIFY isStaleChanged)

public slots:
    /**
     * @brief Sets the topic and creates the ROS subscription.
     * @param topic  ROS topic name; logs a warning if called before node is set.
     */
    void setTopic(QString topic) {
        topic_ = topic;
        if (getRosNode() == nullptr) {
            qWarning() << "Subscriber topic changed before node was set! " << topic;
            return;
        }
        interfacePtr()->setNode(getNode());
        interfacePtr()->setCallback([this]() { handleMsg(); });
        interfacePtr()->subscribe(topic_, queue_size_, latched_);
        emit topicChanged();
    }

    /// Returns the current topic name from the underlying rclcpp subscription.
    QString getTopic() { return interfacePtr()->getTopic(); }

    /// Sets the QoS queue depth.
    void setQueueSize(int queueSize) {
        if (queue_size_ != queueSize) { queue_size_ = queueSize; emit queueSizeChanged(); }
    }

    /// Returns the current queue depth.
    int getQueueSize() const { return queue_size_; }

    /// Enables or disables latched (transient_local) QoS.
    void setLatched(bool latched) {
        if (latched_ != latched) { latched_ = latched; emit latchedChanged(); }
    }

    /// Returns true if latched QoS is enabled.
    bool isLatched() const { return latched_; }

    /**
     * @brief Sets the staleness timeout and reconfigures the check timer.
     * @param timeout  Seconds without a message before isStale becomes true.
     *                 Pass 0 to disable staleness checking.
     */
    void setStaleTimeout(double timeout) {
        if (stale_timeout_ != timeout) {
            stale_timeout_ = timeout;
            emit staleTimeoutChanged();
            configureStaleTimer();
        }
    }

    /// Returns the staleness timeout in seconds.
    double getStaleTimeout() const { return stale_timeout_; }

    /// Returns true if the data is currently considered stale.
    bool isStale() const { return is_stale_; }

signals:
    void topicChanged();        ///< Emitted when the topic property changes.
    void queueSizeChanged();    ///< Emitted when queueSize changes.
    void latchedChanged();      ///< Emitted when latched changes.
    void staleTimeoutChanged(); ///< Emitted when staleTimeout changes.
    void isStaleChanged();      ///< Emitted when the staleness state changes.
    /// Emitted on every received message.  Concrete subclasses also emit
    /// type-specific signals (e.g. dataChanged()) from onMsgReceived().
    void msgReceived();

protected:
    /// Called internally on each message; updates the stale timer and emits msgReceived().
    virtual void handleMsg() {
        emit msgReceived();
        onMsgReceived();
        last_msg_timestamp_.restart();
        if (is_stale_) { is_stale_ = false; emit isStaleChanged(); }
    }

    /**
     * @brief Override in concrete subclasses to emit type-specific signals.
     *
     * Called after msgReceived() on every incoming message.
     * The message data is available via interfacePtr()->msgBuffer() (or the
     * typed subscriber's msgBuffer() directly in subclass code).
     */
    virtual void onMsgReceived() = 0;

    /**
     * @brief Returns a pointer to the concrete typed subscriber interface.
     *
     * Implemented by every concrete subclass to return its
     * QRosTypedSubscriber<msg_T> member.
     */
    virtual QRosSubscriberInterface* interfacePtr() = 0;

    /// Configures or restarts the staleness check timer based on stale_timeout_.
    void configureStaleTimer() {
        if (stale_check_timer_) stale_check_timer_->stop();
        if (stale_timeout_ > 0) {
            int check_interval = qMin(int(stale_timeout_ * 250), 100);
            stale_check_timer_->start(check_interval);
            if (!last_msg_timestamp_.isValid()) last_msg_timestamp_.start();
        }
    }

    /// Periodic callback: updates is_stale_ and emits isStaleChanged() if the state flips.
    void checkStaleState() {
        if (stale_timeout_ > 0 && last_msg_timestamp_.isValid()) {
            bool should_be_stale = last_msg_timestamp_.elapsed() > int(stale_timeout_ * 1000);
            if (should_be_stale != is_stale_) { is_stale_ = should_be_stale; emit isStaleChanged(); }
        }
    }

    QString topic_;
    int     queue_size_   = 1;
    bool    latched_      = false;
    double  stale_timeout_ = 0;
    bool    is_stale_     = false;
    QElapsedTimer last_msg_timestamp_;
    QTimer* stale_check_timer_ = new QTimer(this);

    QRosSubscriber() {
        connect(this, &QRosObject::nodeChanged, this, [this]() {
            if (!topic_.isEmpty()) setTopic(topic_);
        });
        connect(stale_check_timer_, &QTimer::timeout, this, &QRosSubscriber::checkStaleState);
        last_msg_timestamp_.start();
    }
};

QROS_NS_FOOT
