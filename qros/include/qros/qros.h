#pragma once

#include "qros_node.h"
#include "qqml.h"
#include "qros_string_subscriber.h"
#include "qros_string_publisher.h"
#include "qros_valve_publisher.h"
#include "qros_valve_stamped.h"
#include "qros_valve_pack.h"
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
#include "qros_float32.h"
#include "qros_raw_analog_array.h"
#include "qros_wrench_stamped.h"
#include "qros_thrust_stamped.h"
#include "qros_range.h"


#define REGISTER_QML_TYPE(TYPE) \
  qmlRegisterType<TYPE>(QML_PACKAGE, QML_PACKAGE_VERSION_MAJOR, QML_PACKAGE_VERSION_MINOR, #TYPE); \
  qRegisterMetaType<TYPE*>("const " #TYPE "*");

QROS_NS_HEAD
namespace qros {
void registerQmlTypes(){
  REGISTER_QML_TYPE(QRosNode)

  // std_msgs
  REGISTER_QML_TYPE(QRosStringSubscriber)
  REGISTER_QML_TYPE(QRosStringPublisher)
  REGISTER_QML_TYPE(QRosBoolPublisher)
  REGISTER_QML_TYPE(QRosBoolSubscriber)
  REGISTER_QML_TYPE(QRosFloat32Publisher)
  REGISTER_QML_TYPE(QRosFloat32Subscriber)

  // sensor_msgs
  REGISTER_QML_TYPE(QRosTemperatureSubscriber)
  REGISTER_QML_TYPE(QRosFluidPressureSubscriber)
  REGISTER_QML_TYPE(QRosJoyPublisher)
  REGISTER_QML_TYPE(QRosJoySubscriber)

  // geometry_msgs
  REGISTER_QML_TYPE(QRosPoseStampedPublisher)
  REGISTER_QML_TYPE(QRosPoseStampedSubscriber)
  REGISTER_QML_TYPE(QRosTwistStampedPublisher)
  REGISTER_QML_TYPE(QRosTwistStampedSubscriber)

  // nav_msgs
  REGISTER_QML_TYPE(QRosOdometryPublisher)
  REGISTER_QML_TYPE(QRosOdometrySubscriber)

  // diagnostic_msgs
  REGISTER_QML_TYPE(QRosDiagnosticStatusSubscriber)
  REGISTER_QML_TYPE(QRosDiagnosticArraySubscriber)

  // custom msgs (roship)
  REGISTER_QML_TYPE(QRosValvePublisher)
  REGISTER_QML_TYPE(QRosValveStampedPublisher)
  REGISTER_QML_TYPE(QRosValvePackSubscriber)
  REGISTER_QML_TYPE(QRosValvePackPublisher)
  REGISTER_QML_TYPE(QRosRawPacketPublisher)
  REGISTER_QML_TYPE(QRosRawPacketSubscriber)
  REGISTER_QML_TYPE(QRosRangeSubscriber)
  REGISTER_QML_TYPE(QRosRangePublisher)
  REGISTER_QML_TYPE(QRosThrustStampedSubscriber)
  REGISTER_QML_TYPE(QRosThrustStampedPublisher)
  REGISTER_QML_TYPE(QRosWrenchStampedPublisher)
  REGISTER_QML_TYPE(QRosWrenchStampedSubscriber)
  REGISTER_QML_TYPE(QRosRawAnalogSubscriber)
}
}

QROS_NS_FOOT
