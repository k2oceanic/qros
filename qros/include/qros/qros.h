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
#include "qros_pose_stamped.h"
#include "qros_joy.h"
#include "qros_odometry.h"
#include "qros_bool.h"
#include "qros_twist_stamped.h"
#include "qros_raw_packet.h"
#include "qros_settings.h"



#define REGISTER_QML_TYPE(TYPE) \
  qmlRegisterType<TYPE>(QML_PACKAGE, QML_PACKAGE_VERSION_MAJOR, QML_PACKAGE_VERSION_MINOR, #TYPE); \
  qRegisterMetaType<TYPE*>("const " #TYPE "*");

QROS_NS_HEAD
namespace qros {
void registerQmlTypes(){
  REGISTER_QML_TYPE(QRosNode)
  REGISTER_QML_TYPE(QRosStringSubscriber)
  REGISTER_QML_TYPE(QRosStringPublisher)
  REGISTER_QML_TYPE(QRosValvePublisher)
  REGISTER_QML_TYPE(QRosTemperatureSubscriber)
  REGISTER_QML_TYPE(QRosFluidPressureSubscriber)
  REGISTER_QML_TYPE(QRosDiagnosticStatusSubscriber)
  REGISTER_QML_TYPE(QRosDiagnosticArraySubscriber)
  REGISTER_QML_TYPE(QRosPoseStampedPublisher)
  REGISTER_QML_TYPE(QRosPoseStampedSubscriber)
  REGISTER_QML_TYPE(QRosJoyPublisher)
  REGISTER_QML_TYPE(QRosJoySubscriber)
  REGISTER_QML_TYPE(QRosOdometryPublisher)
  REGISTER_QML_TYPE(QRosOdometrySubscriber)
  REGISTER_QML_TYPE(QRosBoolPublisher)
  REGISTER_QML_TYPE(QRosBoolSubscriber)
  REGISTER_QML_TYPE(QRosTwistStampedPublisher)
  REGISTER_QML_TYPE(QRosTwistStampedSubscriber)
  REGISTER_QML_TYPE(QRosRawPacketPublisher)
  REGISTER_QML_TYPE(QRosRawPacketSubscriber)
}
}

QROS_NS_FOOT
