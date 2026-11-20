#pragma once

#include "package_defs.hpp"

/**
 * @file qros_defs.h
 * @brief Central macro definitions for the qros library.
 *
 * Defines the QML import package name and version used by
 * qmlRegisterType() calls throughout qros.cpp, and the
 * QROS_NS_HEAD / QROS_NS_FOOT namespace guards (currently
 * transparent, reserved for future namespace isolation).
 */

/// QML import URI — `import QRos 1.0` in every QML file that uses qros types.
#define QML_PACKAGE "QRos"

/// Major version component of the QML module.
#define QML_PACKAGE_VERSION_MAJOR 1

/// Minor version component of the QML module.
#define QML_PACKAGE_VERSION_MINOR 0

/// @cond INTERNAL
/// Opening guard for the (currently transparent) qros namespace.
#define QROS_NS_HEAD  // NS_HEAD namespace qros {

/// Closing guard for the (currently transparent) qros namespace.
#define QROS_NS_FOOT  // NS_FOOT }
/// @endcond
