#include "qros_register.hpp"

#include "qros/qros_pose_stamped.h"
#include "qros/qros_twist_stamped.h"
#include "qros/qros_point_stamped.h"
#include "qros/qros_odometry.h"
#include "qros/qros_geo_point.h"
#include "qros/qros_wrench_stamped.h"

QROS_NS_HEAD
namespace qros {
void registerGeometryMsgsQmlTypes(){
  REGISTER_QML_TYPE(QRosPoseStampedPublisher)
  REGISTER_QML_TYPE(QRosPoseStampedSubscriber)
  REGISTER_QML_TYPE(QRosTwistStampedPublisher)
  REGISTER_QML_TYPE(QRosTwistStampedSubscriber)
  REGISTER_QML_TYPE(QRosPointStampedPublisher)
  REGISTER_QML_TYPE(QRosPointStampedSubscriber)
  REGISTER_QML_TYPE(QRosOdometryPublisher)
  REGISTER_QML_TYPE(QRosOdometrySubscriber)
  REGISTER_QML_TYPE(QRosGeoPointPublisher)
  REGISTER_QML_TYPE(QRosGeoPointSubscriber)
  REGISTER_QML_TYPE(QRosWrenchStampedPublisher)
  REGISTER_QML_TYPE(QRosWrenchStampedSubscriber)
}
}
QROS_NS_FOOT
