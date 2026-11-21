#include "qros_register.hpp"

#include "qros/qros_temperature_subscriber.h"
#include "qros/qros_pressure_subscriber.h"
#include "qros/qros_joy.h"
#include "qros/qros_joint_state.h"
#include "qros/qros_imu.h"
#include "qros/qros_navsat_fix.h"
#include "qros/qros_range.h"

QROS_NS_HEAD
namespace qros {
void registerSensorMsgsQmlTypes(){
  REGISTER_QML_TYPE(QRosTemperatureSubscriber)
  REGISTER_QML_TYPE(QRosFluidPressureSubscriber)
  REGISTER_QML_TYPE(QRosJoyPublisher)
  REGISTER_QML_TYPE(QRosJoySubscriber)
  REGISTER_QML_TYPE(QRosJointStatePublisher)
  REGISTER_QML_TYPE(QRosJointStateSubscriber)
  REGISTER_QML_TYPE(QRosImuPublisher)
  REGISTER_QML_TYPE(QRosImuSubscriber)
  REGISTER_QML_TYPE(QRosNavSatFixPublisher)
  REGISTER_QML_TYPE(QRosNavSatFixSubscriber)
  REGISTER_QML_TYPE(QRosRangeSubscriber)
  REGISTER_QML_TYPE(QRosRangePublisher)
}
}
QROS_NS_FOOT
