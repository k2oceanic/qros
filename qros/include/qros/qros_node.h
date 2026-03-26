
#ifndef QROS_NODE_H
#define QROS_NODE_H

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

QROS_NS_HEAD
class QRosNode : public QObject{
  Q_OBJECT
public:
  Q_PROPERTY(QVariantMap parameters READ getParameters NOTIFY parametersChanged)
  Q_PROPERTY(QRosParameterEvent* parameterEvent READ getParameterEvent CONSTANT)

  explicit QRosNode(QObject *parent = nullptr);
  rclcpp::Node::SharedPtr getNodePtr() const;
  void setNodePtr(const rclcpp::Node::SharedPtr &newNode_ptr);
  QRosParameterEvent* getParameterEvent() const;
  rclcpp::ParameterValue paramValueFromQVariant(const QVariant &value);
  QVariant arrayToVariantList(const rclcpp::Parameter &param);
  QVariant paramValueToQVariant(const rclcpp::Parameter &param);

public slots:
  void spinRosWithTimer(int durration = 10);
  void spinRosSome();
  /*!
   * \brief getTopicsOfType lists all the topics that are of the specified type
   * \param topic_type a that specifies the topic type i.e. std_msgs/msg/Bool
   * \return a QStringList of all the topics corresponding to the requested topic type
   */
  QStringList getTopicsOfType(QString topic_type);

  void declareParameter(const QString &param_name, const QVariant &default_value);
  QVariantMap getParameters();
  void updateParameters();
  void setParameter(const QString &param_name, const QVariant &value);
  void setExternalParameterAsync(const QString &node_name, const QString &param_name, const QVariant &value, int wait_ms = 1000);
  void setExternalParameter(const QString &node_name, const QString &param_name, const QVariant &value, int wait_ms = 1000);
  void getExternalParametersAsync(const QString &node_name, const QStringList &param_names, int wait_ms = 1000);
  void getExternalParameters(const QString &node_name, const QStringList &param_names, int wait_ms = 1000);
  void listExternalParametersAsync(const QString &node_name, int wait_ms = 1000);
  int  countSubscribers(const QString &topic);
  int  countPublishers(const QString &topic);
  QString getName();


signals:
  void parametersChanged();
  void nodeChanged();
  void parameterSetResult(bool result, QString node_name, QString param_name);
  void parametersGetResult(bool success, QString node_name, QVariantMap params, QString error = "");
  void parametersListResult(bool success, QString node_name, QStringList param_names, QString error = "");
private:
  rclcpp::Node::SharedPtr node_ptr_ = nullptr;
  QTimer *ros_timer_;
  QVariantMap parameters_;
  QRosParameterEvent *parameter_event_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr param_event_sub_;
};
QROS_NS_FOOT
#endif
