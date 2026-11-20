#pragma once

/**
 * @file qros_register.hpp
 * @brief Private helpers for qros::registerQmlTypes().
 *
 * QML registration is split across several translation units, one per message
 * group. Registering every type in a single file instantiates the rclcpp
 * templates for all of them at once and needs ~5GB RAM to compile.
 */

#include <qqml.h>

#include "qros/qros_defs.h"

#ifndef REGISTER_QML_TYPE
#define REGISTER_QML_TYPE(TYPE) \
  qmlRegisterType<TYPE>(QML_PACKAGE, QML_PACKAGE_VERSION_MAJOR, QML_PACKAGE_VERSION_MINOR, #TYPE); \
  qRegisterMetaType<TYPE*>("const " #TYPE "*");
#endif

QROS_NS_HEAD
namespace qros {
void registerCoreQmlTypes();
void registerStdMsgsQmlTypes();
void registerSensorMsgsQmlTypes();
void registerGeometryMsgsQmlTypes();
void registerDiagnosticMsgsQmlTypes();
void registerRoshipQmlTypes();
}
QROS_NS_FOOT
