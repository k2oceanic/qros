#pragma once

#include "qros_defs.h"
#include "qros_node.h"

#include <qobject.h>
#include <rclcpp/rclcpp.hpp>


QROS_NS_HEAD
class QRosObject : public QObject{
    Q_OBJECT
    Q_PROPERTY(QRosNode *node READ getNode WRITE setNode NOTIFY nodeChanged FINAL)
public:
    rclcpp::Node::SharedPtr getRosNode();
    QRosNode *getNode() const;
    void setNode(QRosNode *newNode);
signals:
    void nodeChanged();
private:
    QRosNode* node;
};
QROS_NS_FOOT
