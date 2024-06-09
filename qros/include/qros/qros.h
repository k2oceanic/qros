// #pragma once
#ifndef QROS_H
#define QROS_H

#include "qros_node.h"
#include "qqml.h"
#include "qros_publisher.h"
#include "qros_subscriber.h"
#include "qros_string_subscriber.h"
#include "qros_string_publisher.h"
#include "qros_valve_publisher.h"
#include "qros_valve_stamped.h"
#include "qros_valve_pack.h"
#include "qros_temperature_subscriber.h"
#include "qros_pressure_subscriber.h"
#include "qros_diagnostic_status_subscriber.h"
#include "qros_diagnostic_array_subscriber.h"
#include "qros_joint_state.h"
#include "qros_pose_stamped.h"
#include "qros_joy.h"
#include "qros_odometry.h"
#include "qros_primitives.h"
#include "qros_twist_stamped.h"
#include "qros_raw_packet.h"
#include "qros_settings.h"
#include "qros_float32.h"
#include "qros_raw_analog_array.h"
#include "qros_wrench_stamped.h"
#include "qros_thrust_stamped.h"
#include "qros_range.h"
#include "qros_imu.h"

#define REGISTER_QML_TYPE(TYPE) \
  qmlRegisterType<TYPE>(QML_PACKAGE, QML_PACKAGE_VERSION_MAJOR, QML_PACKAGE_VERSION_MINOR, #TYPE); \
  qRegisterMetaType<TYPE*>("const " #TYPE "*");

QROS_NS_HEAD
namespace qros {
void registerQmlTypes(); // Declaration
}
QROS_NS_FOOT

#endif
