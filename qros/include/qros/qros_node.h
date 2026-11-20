
#ifndef QROS_NODE_H
#define QROS_NODE_H

/**
 * @file qros_node.h
 * @brief Central ROS 2 node wrapper for QML applications.
 */

#include "qros_defs.h"
#include "qros_parameter_event.h"

#include <QObject>
#include <QTimer>
#include <QStringList>
#include <QVariant>
#include <QStringList>
#include <QVariantMap>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_event.hpp>

#include <mutex>
#include <unordered_map>

QROS_NS_HEAD

/**
 * @brief QML-exposed wrapper around a single rclcpp::Node.
 *
 * QRosNode is the entry point for every qros-based application.  It is
 * created in C++ main(), attached to a pre-existing rclcpp node via
 * setNodePtr(), and then exposed to QML as a context property
 * (conventionally named @c applicationNode).
 *
 * All publishers, subscribers, service clients, and TF buffers take a
 * `node: applicationNode` property that points back to this object.
 *
 * ### ROS spinning
 * Call spinRosWithTimer() once after setNodePtr().  It starts an internal
 * QTimer that drives rclcpp::spin_some() at the requested interval, keeping
 * ROS callbacks executing on the Qt main thread.
 *
 * ### Parameter support
 * QRosNode exposes both local node parameters (declareParameter, setParameter,
 * getParameters) and remote parameter operations (setExternalParameter,
 * getExternalParameters, listExternalParameters).  All remote calls have both
 * synchronous and async variants; async variants emit completion signals.
 *
 * ### C++ setup
 * @code{.cpp}
 * auto ros_node = std::make_shared<rclcpp::Node>("my_app");
 * QRosNode applicationNode;
 * applicationNode.setNodePtr(ros_node);
 * applicationNode.spinRosWithTimer(10);   // 10 ms ≈ 100 Hz
 * engine.rootContext()->setContextProperty("applicationNode", &applicationNode);
 * @endcode
 *
 * ### QML usage
 * @code{.qml}
 * QRosDoubleSubscriber { node: applicationNode; topic: "/depth" }
 * @endcode
 */
class QRosNode : public QObject{
  Q_OBJECT
  /// Current node parameter map.  Updated by updateParameters() and parameter events.
  Q_PROPERTY(QVariantMap parameters READ getParameters NOTIFY parametersChanged)
  /// Live parameter event stream; connect to its signals to watch any node's parameters.
  Q_PROPERTY(QRosParameterEvent* parameterEvent READ getParameterEvent CONSTANT)

public:
  explicit QRosNode(QObject *parent = nullptr);

  /// Returns the underlying rclcpp node shared pointer.
  rclcpp::Node::SharedPtr getNodePtr() const;

  /**
   * @brief Attaches this wrapper to an existing rclcpp node.
   * @param newNode_ptr  Shared pointer to a fully constructed rclcpp::Node.
   *                     Must be called before any subscribers or publishers are created.
   */
  void setNodePtr(const rclcpp::Node::SharedPtr &newNode_ptr);

  /// Returns the QRosParameterEvent object that broadcasts /parameter_events messages.
  QRosParameterEvent* getParameterEvent() const;

  /// Converts a QVariant to an rclcpp::ParameterValue (used internally).
  rclcpp::ParameterValue paramValueFromQVariant(const QVariant &value);

  /// Converts an array-type rclcpp::Parameter to a QVariantList (used internally).
  QVariant arrayToVariantList(const rclcpp::Parameter &param);

  /// Converts any rclcpp::Parameter to a QVariant (used internally).
  QVariant paramValueToQVariant(const rclcpp::Parameter &param);

public slots:
  /**
   * @brief Starts ROS spinning via a QTimer on the Qt main thread.
   * @param durration  Timer interval in milliseconds (default 10 ms ≈ 100 Hz).
   *
   * Recommended integration method — all subscriber callbacks execute on the
   * Qt main thread, eliminating the need for explicit thread synchronisation.
   */
  void spinRosWithTimer(int durration = 10);

  /// Calls rclcpp::spin_some() once.  Used internally by the timer.
  void spinRosSome();

  /// Returns all currently advertised topic names.
  QStringList getTopics();

  /**
   * @brief Returns all topic names whose type matches @p topic_type.
   * @param topic_type  Full ROS 2 type string, e.g. @c "std_msgs/msg/Bool".
   */
  QStringList getTopicsOfType(QString topic_type);

  /// Returns all currently available service names.
  QStringList getServices();

  /**
   * @brief Returns all service names matching @p service_type.
   * @param service_type  Full ROS 2 type string, e.g. @c "std_srvs/srv/Trigger".
   */
  QStringList getServicesOfType(QString service_type);

  /// Returns the names of all live ROS nodes visible on the network.
  QStringList getNodeNames();

  /**
   * @brief Declares a local node parameter with a default value.
   * @param param_name     Parameter name.
   * @param default_value  Default value (int, double, bool, or QString).
   */
  void declareParameter(const QString &param_name, const QVariant &default_value);

  /// Returns all local node parameters as a QVariantMap.
  QVariantMap getParameters();

  /// Fetches current parameter values from the node and updates the parameters property.
  void updateParameters();

  /**
   * @brief Sets a local node parameter.
   * @param param_name  Parameter name.
   * @param value       New value as a QVariant.
   */
  void setParameter(const QString &param_name, const QVariant &value);

  /**
   * @brief Asynchronously sets a parameter on another node.
   *
   * Result is reported via parameterSetResult().
   * @param node_name   Fully-qualified target node name (e.g. "/thruster_driver").
   * @param param_name  Parameter name on the target node.
   * @param value       New value.
   * @param wait_ms     Timeout in milliseconds (default 1000).
   */
  void setExternalParameterAsync(const QString &node_name, const QString &param_name, const QVariant &value, int wait_ms = 1000);

  /// Synchronous variant of setExternalParameterAsync().
  void setExternalParameter(const QString &node_name, const QString &param_name, const QVariant &value, int wait_ms = 1000);

  /**
   * @brief Asynchronously fetches parameters from another node.
   *
   * Result is reported via parametersGetResult().
   * @param node_name   Fully-qualified target node name.
   * @param param_names Parameter names to fetch.
   * @param wait_ms     Timeout in milliseconds (default 1000).
   */
  void getExternalParametersAsync(const QString &node_name, const QStringList &param_names, int wait_ms = 1000);

  /// Synchronous variant of getExternalParametersAsync().
  void getExternalParameters(const QString &node_name, const QStringList &param_names, int wait_ms = 1000);

  /**
   * @brief Asynchronously lists all parameters on another node.
   *
   * Result is reported via parametersListResult().
   * @param node_name  Fully-qualified target node name.
   * @param wait_ms    Timeout in milliseconds (default 1000).
   */
  void listExternalParametersAsync(const QString &node_name, int wait_ms = 1000);

  /**
   * @brief Returns the number of subscribers on @p topic.
   * @return 0 if the topic is unknown.
   */
  int  countSubscribers(const QString &topic);

  /**
   * @brief Returns the number of publishers on @p topic.
   * @return 0 if the topic is unknown.
   */
  int  countPublishers(const QString &topic);

  /// Returns this node's unqualified name (e.g. "roship_engineering").
  QString getName();

  /// Returns this node's namespace (e.g. "/rov").
  QString getNamespace();

signals:
  /// Emitted when the local parameters map changes.
  void parametersChanged();

  /// Emitted when the rclcpp node pointer changes (setNodePtr was called).
  void nodeChanged();

  /**
   * @brief Result of a set-external-parameter call.
   * @param result     True if the set succeeded.
   * @param node_name  Target node name.
   * @param param_name Parameter that was set.
   */
  void parameterSetResult(bool result, QString node_name, QString param_name);

  /**
   * @brief Result of a get-external-parameters call.
   * @param success    True if the fetch succeeded.
   * @param node_name  Source node name.
   * @param params     Parameter name → value map (empty on failure).
   * @param error      Error description (empty on success).
   */
  void parametersGetResult(bool success, QString node_name, QVariantMap params, QString error = "");

  /**
   * @brief Result of a list-external-parameters call.
   * @param success      True if the list succeeded.
   * @param node_name    Target node name.
   * @param param_names  Parameter names (empty on failure).
   * @param error        Error description (empty on success).
   */
  void parametersListResult(bool success, QString node_name, QStringList param_names, QString error = "");

private:
  std::shared_ptr<rclcpp::AsyncParametersClient> getOrCreateParamClient(const std::string &node_name);

  rclcpp::Node::SharedPtr node_ptr_ = nullptr;
  QTimer *ros_timer_;
  QVariantMap parameters_;
  QRosParameterEvent *parameter_event_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr param_event_sub_;

  std::unordered_map<std::string, std::shared_ptr<rclcpp::AsyncParametersClient>> param_clients_;
  std::mutex param_clients_mutex_;
};
QROS_NS_FOOT
#endif
