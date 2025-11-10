#include "qros_object.h"

QROS_NS_HEAD


rclcpp::Node::SharedPtr QRosObject::getRosNode()
{
  if(node){
    return node->getNodePtr();
  }else{
    return nullptr;
  }
}

QRosNode *QRosObject::getNode() const
{
  if(node){
    return node;
  }else{
    return nullptr;
  }
}

void QRosObject::setNode(QRosNode *newNode)
{
    if (node == newNode)
        return;
    node = newNode;
    emit nodeChanged();
}



QROS_NS_FOOT
