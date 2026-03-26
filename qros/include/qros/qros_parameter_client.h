#pragma once

#include "qros_object.h"
#include "qros_parameter_event.h"

#include <QVariant>
#include <QString>

QROS_NS_HEAD
/*!
 * \brief QRosParameterClient binds to a single parameter on an external ROS node.
 *
 * Fetches the initial value via getExternalParametersAsync and stays live via
 * parameterEvent. Can also write back to the parameter service, making it a
 * full client of the remote node's parameter server.
 *
 * override (default true): when the driver restarts and freshly declares a parameter,
 * push the last known value back instead of accepting the driver's YAML default.
 * Set override: false to let the driver own its state on restart.
 *
 * QML usage — UI overrides driver on restart (default):
 *   QRosParameterClient {
 *       node: applicationNode
 *       watchedNode: "/ixys_driver"
 *       watchedParam: "node_1.trip_level_amps"
 *       onValueChanged: cachedTripLevels = value
 *   }
 *
 * QML usage — driver owns state on restart:
 *   QRosParameterClient {
 *       node: applicationNode
 *       watchedNode: "/some_node"
 *       watchedParam: "my_param"
 *       override: false
 *       onNewParam: console.log("node restarted, driver value:", value)
 *       onValueChanged: myDisplay.value = value
 *   }
 */
class QRosParameterClient : public QRosObject {
  Q_OBJECT
  Q_PROPERTY(QString  watchedNode  READ getWatchedNode  WRITE setWatchedNode  NOTIFY watchedNodeChanged)
  Q_PROPERTY(QString  watchedParam READ getWatchedParam WRITE setWatchedParam NOTIFY watchedParamChanged)
  Q_PROPERTY(QVariant value        READ getValue                               NOTIFY valueChanged)
  Q_PROPERTY(bool     available    READ isAvailable                            NOTIFY availableChanged)
  Q_PROPERTY(bool     override     READ isOverride      WRITE setOverride      NOTIFY overrideChanged)
public:
  explicit QRosParameterClient(QObject *parent = nullptr);

  QString  getWatchedNode()  const { return watched_node_; }
  QString  getWatchedParam() const { return watched_param_; }
  QVariant getValue()        const { return value_; }
  bool     isAvailable()     const { return available_; }
  bool     isOverride()      const { return override_; }

  void setWatchedNode (const QString &node);
  void setWatchedParam(const QString &param);
  void setOverride    (bool enable);
  Q_INVOKABLE void set(const QVariant &value);

signals:
  void watchedNodeChanged();
  void watchedParamChanged();
  void valueChanged();
  void availableChanged();
  void overrideChanged();
  /*! Emitted when the watched node restarts and freshly declares the parameter.
   *  \param value  the driver's initial value (from its YAML config).
   *  When override is true this value is ignored and the latched value is pushed back.
   *  When override is false this value is accepted and exposed via the value property. */
  void newParam(QVariant value);

private:
  void reconnect();
  void fetch();
  void setAvailable(bool a);
  void setValue(const QVariant &v);

  QString  watched_node_;
  QString  watched_param_;
  QVariant value_;
  bool     available_ = false;
  bool     override_  = true;

  QMetaObject::Connection conn_params_result_;
  QMetaObject::Connection conn_new_param_;
  QMetaObject::Connection conn_param_event_;
  QMetaObject::Connection conn_deleted_;
};
QROS_NS_FOOT
