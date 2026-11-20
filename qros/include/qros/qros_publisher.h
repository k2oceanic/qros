#pragma once

/**
 * @file qros_publisher.h
 * @brief Publisher base classes for the qros library.
 *
 * Provides two layers:
 *  - QRosTypedPublisher<msg_T>  — type-erased C++ template that owns the
 *    rclcpp publisher and message buffer.
 *  - QRosPublisher              — QObject base exposed to QML; subclasses
 *    return a QRosTypedPublisher via interfacePtr().
 */

#include "qdebug.h"
#include "qros_object.h"

QROS_NS_HEAD

/**
 * @brief Abstract interface for all typed publishers.
 *
 * Provides a uniform, type-erased API so QRosPublisher can drive any
 * message type without knowing it at compile time.
 */
class QRosPublisherInterface {
public:
    typedef std::shared_ptr<QRosPublisherInterface> SharedPtr;

    /// Binds the publisher to the rclcpp node held by @p node.
    virtual void setNode(QRosNode* node) = 0;

    /// Publishes the current contents of msgBuffer().
    virtual void publish() = 0;

    /**
     * @brief Creates the underlying rclcpp publisher.
     * @param topic     ROS topic name.
     * @param queue_size QoS history depth.
     * @param latched   If true, uses transient_local (latched) QoS.
     */
    virtual void createRosPub(QString topic, int queue_size, bool latched) = 0;

    /// Returns the fully-resolved topic name.
    virtual QString getTopic() = 0;
};

/**
 * @brief Concrete, type-specific publisher template.
 *
 * Owns the rclcpp::Publisher<msg_T> and a single message buffer.
 * Call msgBuffer() to populate fields, then publish() to send.
 *
 * @tparam msg_T  ROS 2 message type (e.g. std_msgs::msg::Float64).
 */
template <typename msg_T>
class QRosTypedPublisher : public QRosPublisherInterface {
public:
    /// @copydoc QRosPublisherInterface::setNode
    void setNode(QRosNode* node) {
        ros_node_ptr_ = node->getNodePtr();
    }

    /// Publishes msg_buffer_ if the publisher is valid.
    void publish() {
        if (ros_pub_)
            ros_pub_->publish(msg_buffer_);
    }

    /**
     * @brief Creates the rclcpp publisher with the given QoS settings.
     * @param topic      ROS topic name.
     * @param queue_size History depth (default 10).
     * @param latched    Transient-local if true, volatile if false (default false).
     */
    void createRosPub(QString topic, int queue_size = 10, bool latched = false) {
        auto qos = latched
            ? rclcpp::QoS(rclcpp::KeepLast(queue_size)).transient_local()
            : rclcpp::QoS(queue_size);
        ros_pub_ = ros_node_ptr_->template create_publisher<msg_T>(topic.toStdString(), qos);
    }

    /// Returns the fully-resolved topic name from the underlying rclcpp publisher.
    QString getTopic() {
        return QString::fromStdString(ros_pub_->get_topic_name());
    }

    /**
     * @brief Returns a mutable reference to the outgoing message buffer.
     *
     * Populate fields on this reference, then call publish().
     */
    msg_T& msgBuffer() { return msg_buffer_; }

    /// Returns a const copy of the message buffer.
    const msg_T getConstBuffer() { return msg_buffer_; }

private:
    msg_T msg_buffer_;
    rclcpp::Node::SharedPtr ros_node_ptr_;
    typename rclcpp::Publisher<msg_T>::SharedPtr ros_pub_;
};

/**
 * @brief QML-exposed base class for all qros publishers.
 *
 * Concrete subclasses (e.g. QRosDoublePublisher) implement interfacePtr()
 * to return their QRosTypedPublisher<msg_T>, and expose message fields as
 * additional Q_PROPERTYs.
 *
 * ### QML usage
 * @code{.qml}
 * QRosDoublePublisher {
 *     id:    depthPub
 *     node:  applicationNode
 *     topic: "/commanded_depth"
 * }
 * Button {
 *     onClicked: { depthPub.data = 5.0; depthPub.publish() }
 * }
 * @endcode
 */
class QRosPublisher : public QRosObject {
    Q_OBJECT
public:
    /// ROS topic to publish on.  Setting this creates the rclcpp publisher.
    Q_PROPERTY(QString topic     READ getTopic     WRITE setTopic     NOTIFY topicChanged)
    /// QoS history depth.  Must be set before topic for the setting to take effect.
    Q_PROPERTY(int     queueSize READ getQueueSize WRITE setQueueSize NOTIFY queueSizeChanged)
    /// If true, uses transient_local (latched) QoS so late subscribers receive the last message.
    Q_PROPERTY(bool    latched   READ isLatched    WRITE setLatched   NOTIFY latchedChanged)

public slots:
    /**
     * @brief Sets the topic and creates the ROS publisher.
     * @param topic  ROS topic name; empty string is silently ignored.
     */
    void setTopic(QString topic) {
        if (topic.isEmpty()) return;
        topic_ = topic;
        if (getNode() == nullptr) return;
        try {
            interfacePtr()->setNode(getNode());
            interfacePtr()->createRosPub(topic_, queue_size_, latched_);
            emit topicChanged();
        } catch (...) {
            qWarning() << "invalid publisher topic name: " << topic;
        }
    }

    /// Returns the current topic string.
    QString getTopic() const { return topic_; }

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

    /// Publishes the current message buffer via the typed interface.
    void publish() {
        auto interface = interfacePtr();
        if (interface) {
            interfacePtr()->publish();
        } else {
            RCLCPP_ERROR(getRosNode()->get_logger(),
                         "Invalid interface pointer passed to QRosPublisher::publish()");
        }
    }

signals:
    void topicChanged();     ///< Emitted when the topic property changes.
    void queueSizeChanged(); ///< Emitted when queueSize changes.
    void latchedChanged();   ///< Emitted when latched changes.

protected:
    QRosPublisher() {
        connect(this, &QRosObject::nodeChanged, this, [this]() {
            if (!topic_.isEmpty()) setTopic(topic_);
        });
    }

    /**
     * @brief Returns a pointer to the concrete typed publisher interface.
     *
     * Implemented by every concrete subclass to return its
     * QRosTypedPublisher<msg_T> member.
     */
    virtual QRosPublisherInterface* interfacePtr() = 0;

    QString topic_;
    int     queue_size_ = 10;
    bool    latched_    = false;
};

QROS_NS_FOOT
