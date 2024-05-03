
#ifndef QROS_NODE_H
#define QROS_NODE_H

#include "qros_defs.h"

#include <QObject>
#include <QTimer>
#include <QStringList>
#include <QVariant>
#include <QStringList>
#include <QVariantMap>

#include <rclcpp/rclcpp.hpp>

QROS_NS_HEAD
class QRosNode : public QObject{
  Q_OBJECT
public:
  Q_PROPERTY(QVariantMap parameters READ getParameters NOTIFY parametersChanged)

  explicit QRosNode(QObject *parent = nullptr);
  rclcpp::Node::SharedPtr getNodePtr() const;
  void setNodePtr(const rclcpp::Node::SharedPtr &newNode_ptr);
  rclcpp::ParameterValue paramValueFromQVariant(const QVariant &value);

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

signals:
  void parametersChanged();
  void parameterSetResult(bool result, QString node_name, QString param_name);

private:
  rclcpp::Node::SharedPtr node_ptr_;
  QTimer *ros_timer_;
  QVariantMap parameters_;
};
QROS_NS_FOOT
#endif
