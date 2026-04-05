#pragma once

#include "qros_defs.h"

#include <QObject>
#include <QString>
#include <QVariant>

QROS_NS_HEAD
class QRosParameterEvent : public QObject {
  Q_OBJECT
public:
  explicit QRosParameterEvent(QObject *parent = nullptr) : QObject(parent) {}

signals:
  /*!
   * \brief event emitted whenever any ROS parameter event is received on /parameter_events
   * \param node_name  fully-qualified name of the node whose parameter changed
   * \param param_name name of the parameter
   * \param value      new value of the parameter as a QVariant
   *
   * QML usage:
   *   Connections {
   *     target: applicationNode.parameterEvent
   *     function onEvent(nodeName, paramName, value) {
   *       if (nodeName === "/my_node" && paramName === "my_param") { ... }
   *     }
   *   }
   */
  void event(QString node_name, QString param_name, QVariant value);       // changed_parameters
  void newParam(QString node_name, QString param_name, QVariant value);   // new_parameters (node started/restarted)
  void deleted(QString node_name, QString param_name);                    // deleted_parameters (node shut down)
};
QROS_NS_FOOT
