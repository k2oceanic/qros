#include "qros_register.hpp"

#include "qros/qros_valve_publisher.h"
#include "qros/qros_valve_stamped.h"
#include "qros/qros_valve_pack.h"
#include "qros/qros_raw_packet.h"
#include "qros/qros_thrust_stamped.h"
#include "qros/qros_thrust_array.h"
#include "qros/qros_raw_analog_array.h"
#include "qros/qros_raw_digital.h"
#include "qros/qros_channel_trigger_client.h"

QROS_NS_HEAD
namespace qros {
void registerRoshipQmlTypes(){
  REGISTER_QML_TYPE(QRosValvePublisher)
  REGISTER_QML_TYPE(QRosValveStampedPublisher)
  REGISTER_QML_TYPE(QRosValvePackSubscriber)
  REGISTER_QML_TYPE(QRosValvePackPublisher)
  REGISTER_QML_TYPE(QRosRawPacketPublisher)
  REGISTER_QML_TYPE(QRosRawPacketSubscriber)
  REGISTER_QML_TYPE(QRosThrustStampedSubscriber)
  REGISTER_QML_TYPE(QRosThrustStampedPublisher)
  REGISTER_QML_TYPE(QRosThrustArraySubscriber)
  REGISTER_QML_TYPE(QRosThrustArrayPublisher)
  REGISTER_QML_TYPE(QRosRawAnalogSubscriber)
  REGISTER_QML_TYPE(QRosRawDigitalArrayPublisher)
  REGISTER_QML_TYPE(QRosRawDigitalArraySubscriber)
  REGISTER_QML_TYPE(QRosChannelTriggerClient)
}
}
QROS_NS_FOOT
