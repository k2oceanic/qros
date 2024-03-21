#include "qros_node.h"

QROS_NS_HEAD

QRosNode::QRosNode(QObject *parent)
    : QObject{parent}
{}


rclcpp::Node::SharedPtr QRosNode::getNodePtr() const
{
    return node_ptr_;
}

void QRosNode::setNodePtr(const rclcpp::Node::SharedPtr &newNode_ptr)
{
  node_ptr_ = newNode_ptr;
}

void QRosNode::spinRosWithTimer(int durration)
{
  ros_timer_ = new QTimer(this);
  connect(ros_timer_, SIGNAL(timeout()), this, SLOT(spinRosSome()));
  ros_timer_->start(durration);
}

void QRosNode::spinRosSome()
{
  rclcpp::spin_some(node_ptr_);
}

QROS_NS_FOOT
