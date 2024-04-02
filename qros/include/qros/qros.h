#pragma once

#include "qros_node.h"
#include "qqml.h"
#include "qros_string_subscriber.h"
#include "qros_string_publisher.h"
#include "qros_valve_publisher.h"
#include "qros_temperature_subscriber.h"
#include "qros_pressure_subscriber.h"
#include "qros_diagnostic_status_subscriber.h"
#include "qros_diagnostic_array_subscriber.h"

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

  qmlRegisterType<QRosValvePublisher> (QML_PACKAGE, QML_PACKAGE_VERSION_MAJOR,QML_PACKAGE_VERSION_MINOR,"QRosValvePublisher");
  qRegisterMetaType<QRosValvePublisher*>("const QRosValvePublisher*");

  qmlRegisterType<QRosTemperatureSubscriber> (QML_PACKAGE, QML_PACKAGE_VERSION_MAJOR,QML_PACKAGE_VERSION_MINOR,"QRosTemperatureSubscriber");
  qRegisterMetaType<QRosTemperatureSubscriber*>("const QRosTemperatureSubscriber*");

  qmlRegisterType<QRosFluidPressureSubscriber> (QML_PACKAGE, QML_PACKAGE_VERSION_MAJOR,QML_PACKAGE_VERSION_MINOR,"QRosFluidPressureSubscriber");
  qRegisterMetaType<QRosFluidPressureSubscriber*>("const QRosFluidPressureSubscriber*");

  qmlRegisterType<QRosDiagnosticStatusSubscriber> (QML_PACKAGE, QML_PACKAGE_VERSION_MAJOR,QML_PACKAGE_VERSION_MINOR,"QRosDiagnosticStatusSubscriber");
  qRegisterMetaType<QRosDiagnosticStatusSubscriber*>("const QRosDiagnosticStatusSubscriber*");

  qmlRegisterType<QRosDiagnosticArraySubscriber> (QML_PACKAGE, QML_PACKAGE_VERSION_MAJOR,QML_PACKAGE_VERSION_MINOR,"QRosDiagnosticArraySubscriber");
  qRegisterMetaType<QRosDiagnosticArraySubscriber*>("const QRosDiagnosticArraySubscriber*");

  qmlRegisterType<QRosDiagnosticArraySubscriber> (QML_PACKAGE, QML_PACKAGE_VERSION_MAJOR,QML_PACKAGE_VERSION_MINOR,"QRosDiagnosticArraySubscriber");
  qRegisterMetaType<QRosDiagnosticArraySubscriber*>("const QRosDiagnosticArraySubscriber*");

  qmlRegisterType<QRosDiagnosticArraySubscriber> (QML_PACKAGE, QML_PACKAGE_VERSION_MAJOR,QML_PACKAGE_VERSION_MINOR,"QRosHeader");
  qRegisterMetaType<QRosDiagnosticArraySubscriber*>("const QRosHeader*");

  qmlRegisterType<QRosDiagnosticArraySubscriber> (QML_PACKAGE, QML_PACKAGE_VERSION_MAJOR,QML_PACKAGE_VERSION_MINOR,"QRosHeaderPublisher");
  qRegisterMetaType<QRosDiagnosticArraySubscriber*>("const QRosHeaderPublisher*");
}

QROS_NS_FOOT
