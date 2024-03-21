
#ifndef QROS_NODE_H
#define QROS_NODE_H

#include "qros_defs.h"

#include <QObject>
#include <QTimer>

#include <rclcpp/rclcpp.hpp>

QROS_NS_HEAD
class QRosNode : public QObject{
  Q_OBJECT
public:
  explicit QRosNode(QObject *parent = nullptr);
  rclcpp::Node::SharedPtr getNodePtr() const;
  void setNodePtr(const rclcpp::Node::SharedPtr &newNode_ptr);

public slots:
  void spinRosWithTimer(int durration = 10);
  void spinRosSome();

private:
  rclcpp::Node::SharedPtr node_ptr_;
  QTimer *ros_timer_;

};
QROS_NS_FOOT
#endif
