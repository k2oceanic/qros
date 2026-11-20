#pragma once

/**
 * @file qros_parameter_client.h
 * @brief QML live-binding client for a single parameter on a remote ROS node.
 */

#include "qros_object.h"
#include "qros_parameter_event.h"

#include <QVariant>
#include <QString>

QROS_NS_HEAD

/**
 * @brief Binds a QML property to a single parameter on an external ROS node.
 *
 * On startup, QRosParameterClient fetches the initial value via
 * `getExternalParametersAsync`.  It then stays live by listening to the
 * `parameterEvent` signal stream and updating `value` whenever the watched
 * parameter changes.
 *
 * The `override` property controls what happens when the watched node
 * restarts and redeclares the parameter with its YAML default:
 *  - `override: true`  (default) — the last known UI value is pushed back to
 *    the driver, overriding the YAML default.  Use this when the UI owns state.
 *  - `override: false` — the driver's initial value is accepted as the new
 *    `value`.  Use this when the driver owns state on restart.
 *
 * ### QML usage — UI overrides driver on restart (default)
 * @code{.qml}
 * QRosParameterClient {
 *     node:         applicationNode
 *     watchedNode:  "/ixys_driver"
 *     watchedParam: "node_1.trip_level_amps"
 *     onValueChanged: tripLevelDisplay.value = value
 * }
 * @endcode
 *
 * ### QML usage — driver owns state on restart
 * @code{.qml}
 * QRosParameterClient {
 *     node:         applicationNode
 *     watchedNode:  "/some_driver"
 *     watchedParam: "gain"
 *     override:     false
 *     onNewParam:   console.log("driver restarted, gain =", value)
 *     onValueChanged: gainSlider.value = value
 * }
 * @endcode
 */
class QRosParameterClient : public QRosObject {
  Q_OBJECT
  /// Fully-qualified name of the node whose parameter is being watched.
  Q_PROPERTY(QString  watchedNode  READ getWatchedNode  WRITE setWatchedNode  NOTIFY watchedNodeChanged)
  /// Name of the parameter on the remote node.
  Q_PROPERTY(QString  watchedParam READ getWatchedParam WRITE setWatchedParam NOTIFY watchedParamChanged)
  /// Current value of the parameter (read-only; updated automatically).
  Q_PROPERTY(QVariant value        READ getValue                               NOTIFY valueChanged)
  /// True when the remote node and parameter are reachable and have been fetched.
  Q_PROPERTY(bool     available    READ isAvailable                            NOTIFY availableChanged)
  /**
   * @brief Whether this client pushes its last value back when the driver restarts.
   *
   * Default is true (UI owns state).  Set to false to let the driver's YAML
   * default take effect after a restart.
   */
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

  /**
   * @brief Writes a new value to the remote parameter.
   * @param value  New value; type must match the parameter's declared type.
   */
  Q_INVOKABLE void set(const QVariant &value);

signals:
  void watchedNodeChanged();
  void watchedParamChanged();
  /// Emitted whenever the watched parameter's value changes.
  void valueChanged();
  void availableChanged();
  void overrideChanged();
  /**
   * @brief Emitted when the watched node restarts and freshly declares the parameter.
   * @param value  The driver's initial value (from its YAML config).
   *
   * When override is true this value is ignored and the latched value is pushed
   * back.  When override is false this value is accepted via the value property.
   */
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
