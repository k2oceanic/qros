#include "qros_register.hpp"

#include "qros/qros_node.h"
#include "qros/qros_parameter_event.h"
#include "qros/qros_parameter_client.h"
#include "qros/qros_service_client.h"
#include "qros/qros_trigger_service.h"
#include "qros/qros_tf_buffer.h"
#include "qros/qros_transform_stamped.h"
#include "qros/qros_qvariant.h"
#include "qros/qros_qvariant_map.h"

QROS_NS_HEAD
namespace qros {
void registerCoreQmlTypes(){
  REGISTER_QML_TYPE(QRosNode)
  REGISTER_QML_TYPE(QRosParameterEvent)
  REGISTER_QML_TYPE(QRosParameterClient)
  REGISTER_QML_TYPE(QRosTriggerServiceClient)
  REGISTER_QML_TYPE(QRosTfBuffer)
  REGISTER_QML_TYPE(QRosTransformStamped)
  REGISTER_QML_TYPE(QRosQVariantPublisher)
  REGISTER_QML_TYPE(QRosQVariantSubscriber)
  REGISTER_QML_TYPE(QRosQVariantMapPublisher)
  REGISTER_QML_TYPE(QRosQVariantMapSubscriber)
}
}
QROS_NS_FOOT
