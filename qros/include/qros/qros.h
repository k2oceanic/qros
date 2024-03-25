#pragma once

#include "qros_node.h"
#include "qqml.h"
#include "qros_string_subscriber.h"
#include "qros_string_publisher.h"

QROS_NS_HEAD

#define QML_PACKAGE "QRos"
#define QML_PACKAGE_VERSION_MAJOR 1
#define QML_PACKAGE_VERSION_MINOR 0

void registerQmlTypes(){
  qmlRegisterType<QRosNode> (QML_PACKAGE, QML_PACKAGE_VERSION_MAJOR,QML_PACKAGE_VERSION_MINOR,"QRosNode");
  qRegisterMetaType<QRosNode*>("const QRosNode*");

  qmlRegisterType<QRosStringSubscriber> (QML_PACKAGE, QML_PACKAGE_VERSION_MAJOR,QML_PACKAGE_VERSION_MINOR,"QRosStringSubscriber");
  qRegisterMetaType<QRosStringSubscriber*>("const QRosStringSubscriber*");

  qmlRegisterType<QRosStringPublisher> (QML_PACKAGE, QML_PACKAGE_VERSION_MAJOR,QML_PACKAGE_VERSION_MINOR,"QRosStringPublisher");
  qRegisterMetaType<QRosStringPublisher*>("const QRosStringPublisher*");
}

QROS_NS_FOOT
