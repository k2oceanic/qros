#include "qros_object.h"

QROS_NS_HEAD


rclcpp::Node::SharedPtr QRosObject::getRosNode()
{
    return node->getNodePtr();
}

QRosNode *QRosObject::getNode() const
{
    return node;
}

void QRosObject::setNode(QRosNode *newNode)
{
    if (node == newNode)
        return;
    node = newNode;
    emit nodeChanged();
}



QROS_NS_FOOT
