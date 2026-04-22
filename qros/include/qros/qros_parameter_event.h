#pragma once

/**
 * @file qros_parameter_event.h
 * @brief Signal bridge for the ROS 2 /parameter_events topic.
 */

#include "qros_defs.h"

#include <QObject>
#include <QString>
#include <QVariant>

QROS_NS_HEAD

/**
 * @brief Emits Qt signals for every change on the ROS 2 /parameter_events topic.
 *
 * Owned by QRosNode and exposed as the `parameterEvent` property.  Connect to
 * its signals to watch parameter changes on any node without polling.
 *
 * Three signals cover the full parameter lifecycle:
 *  - event()    — a parameter was changed on an already-running node
 *  - newParam() — a parameter was declared (node started or restarted)
 *  - deleted()  — a parameter was undeclared (node is shutting down)
 *
 * ### QML usage
 * @code{.qml}
 * Connections {
 *     target: applicationNode.parameterEvent
 *     function onEvent(nodeName, paramName, value) {
 *         if (nodeName === "/thruster_driver" && paramName === "max_rpm")
 *             maxRpmDisplay.value = value
 *     }
 *     function onNewParam(nodeName, paramName, value) {
 *         console.log(nodeName, "declared", paramName, "=", value)
 *     }
 * }
 * @endcode
 */
class QRosParameterEvent : public QObject {
  Q_OBJECT
public:
  explicit QRosParameterEvent(QObject *parent = nullptr) : QObject(parent) {}

signals:
  /**
   * @brief Emitted when a parameter is changed on any live node.
   * @param node_name   Fully-qualified name of the node whose parameter changed.
   * @param param_name  Name of the changed parameter.
   * @param value       New parameter value as a QVariant.
   */
  void event(QString node_name, QString param_name, QVariant value);

  /**
   * @brief Emitted when a parameter is freshly declared (node start / restart).
   * @param node_name   Fully-qualified node name.
   * @param param_name  Newly declared parameter name.
   * @param value       Initial value.
   */
  void newParam(QString node_name, QString param_name, QVariant value);

  /**
   * @brief Emitted when a parameter is undeclared (node shutting down).
   * @param node_name   Fully-qualified node name.
   * @param param_name  Name of the removed parameter.
   */
  void deleted(QString node_name, QString param_name);
};

QROS_NS_FOOT
