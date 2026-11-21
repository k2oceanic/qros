#include "qros_register.hpp"

#include "qros/qros_string_subscriber.h"
#include "qros/qros_string_publisher.h"
#include "qros/qros_primitives.h"
#include "qros/qros_float32.h"

QROS_NS_HEAD
namespace qros {
void registerStdMsgsQmlTypes(){
  REGISTER_QML_TYPE(QRosStringSubscriber)
  REGISTER_QML_TYPE(QRosStringPublisher)
  REGISTER_QML_TYPE(QRosBoolPublisher)
  REGISTER_QML_TYPE(QRosBoolSubscriber)
  REGISTER_QML_TYPE(QRosFloat32Publisher)
  REGISTER_QML_TYPE(QRosFloat32Subscriber)
  REGISTER_QML_TYPE(QRosFloat32MultiArrayPublisher)
  REGISTER_QML_TYPE(QRosFloat32MultiArraySubscriber)
  REGISTER_QML_TYPE(QRosIntPublisher)
  REGISTER_QML_TYPE(QRosIntSubscriber)
  REGISTER_QML_TYPE(QRosInt64Publisher)
  REGISTER_QML_TYPE(QRosInt64Subscriber)
  REGISTER_QML_TYPE(QRosDoublePublisher)
  REGISTER_QML_TYPE(QRosDoubleSubscriber)
}
}
QROS_NS_FOOT
