#include "qros.h"

QROS_NS_HEAD

void registerQmlTypes(){
  qmlRegisterType<QRosNode> (QML_PACKAGE, QML_PACKAGE_VERSION_MAJOR,QML_PACKAGE_VERSION_MINOR,"QRosNode");
  qRegisterMetaType<QRosNode*>("const QRosNode*");

  qmlRegisterType<QRosStringSubscriber> (QML_PACKAGE, QML_PACKAGE_VERSION_MAJOR,QML_PACKAGE_VERSION_MINOR,"QRosStringSubscriber");
  qRegisterMetaType<QRosStringSubscriber*>("const QRosStringSubscriber*");

  qmlRegisterType<QRosStringPublisher> (QML_PACKAGE, QML_PACKAGE_VERSION_MAJOR,QML_PACKAGE_VERSION_MINOR,"QRosStringPublisher");
  qRegisterMetaType<QRosStringPublisher*>("const QRosStringPublisher*");
}

QROS_NS_FOOT