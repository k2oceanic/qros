#pragma once

#include "qros_node.h"
#include "qqml.h"
#include "qros_string_subscriber.h"
#include "qros_string_publisher.h"

QROS_NS_HEAD

#define QML_PACKAGE "QRos"
#define QML_PACKAGE_VERSION_MAJOR 1
#define QML_PACKAGE_VERSION_MINOR 0

struct QRos
{
    int test_int;
};

void registerQmlTypes();

QROS_NS_FOOT
