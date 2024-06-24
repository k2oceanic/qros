#include "qros.h"

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
  REGISTER_QML_TYPE(QRosFloat32MultiArrayPublisher)
  REGISTER_QML_TYPE(QRosFloat32MultiArraySubscriber)
  REGISTER_QML_TYPE(QRosIntPublisher)
  REGISTER_QML_TYPE(QRosIntSubscriber)
  REGISTER_QML_TYPE(QRosDoublePublisher)
  REGISTER_QML_TYPE(QRosDoubleSubscriber)

  // sensor_msgs
  REGISTER_QML_TYPE(QRosTemperatureSubscriber)
  REGISTER_QML_TYPE(QRosFluidPressureSubscriber)
  REGISTER_QML_TYPE(QRosJoyPublisher)
  REGISTER_QML_TYPE(QRosJoySubscriber)
  REGISTER_QML_TYPE(QRosJointStatePublisher)
  REGISTER_QML_TYPE(QRosJointStateSubscriber)
  REGISTER_QML_TYPE(QRosImuPublisher)
  REGISTER_QML_TYPE(QRosImuSubscriber)

  // geometry_msgs
  REGISTER_QML_TYPE(QRosPoseStampedPublisher)
  REGISTER_QML_TYPE(QRosPoseStampedSubscriber)
  REGISTER_QML_TYPE(QRosTwistStampedPublisher)
  REGISTER_QML_TYPE(QRosTwistStampedSubscriber)
  REGISTER_QML_TYPE(QRosPointStampedPublisher)
  REGISTER_QML_TYPE(QRosPointStampedSubscriber)

  // nav_msgs
  REGISTER_QML_TYPE(QRosOdometryPublisher)
  REGISTER_QML_TYPE(QRosOdometrySubscriber)

  // diagnostic_msgs
  REGISTER_QML_TYPE(QRosDiagnosticStatusSubscriber)
  REGISTER_QML_TYPE(QRosDiagnosticStatusPublisher)
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
