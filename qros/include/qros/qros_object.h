#pragma once

/**
 * @file qros_object.h
 * @brief Base class for all QML-exposed ROS objects.
 */

#include "qros_defs.h"
#include "qros_node.h"

#include <qobject.h>
#include <rclcpp/rclcpp.hpp>

QROS_NS_HEAD

/**
 * @brief Base class for all qros QML types that need access to a ROS node.
 *
 * Every publisher, subscriber, service client, and TF buffer inherits from
 * QRosObject.  A single `node` property holds a pointer to the application's
 * QRosNode; concrete subclasses call getRosNode() to obtain the underlying
 * rclcpp::Node::SharedPtr needed to create ROS entities.
 *
 * ### QML usage
 * @code{.qml}
 * QRosDoubleSubscriber {
 *     node:  applicationNode   // required for every qros type
 *     topic: "/my/topic"
 * }
 * @endcode
 */
class QRosObject : public QObject {
    Q_OBJECT

    /**
     * @brief The application's ROS node.
     *
     * Must be set before setting `topic` or `serviceName` on any subclass.
     * Changing `node` after construction resets the underlying ROS entity.
     */
    Q_PROPERTY(QRosNode *node READ getNode WRITE setNode NOTIFY nodeChanged FINAL)

public:
    /// Returns the raw rclcpp node shared pointer held by the assigned QRosNode.
    rclcpp::Node::SharedPtr getRosNode();

    /// Returns the QML-side QRosNode pointer (may be nullptr before assignment).
    QRosNode *getNode() const;

    /// Assigns a QRosNode and emits nodeChanged().
    void setNode(QRosNode *newNode);

signals:
    /// Emitted whenever the node property changes.
    void nodeChanged();

private:
    QRosNode* node;
};

QROS_NS_FOOT
