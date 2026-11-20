#include "qros/qros.h"
#include "qros_register.hpp"

QROS_NS_HEAD
namespace qros {
void registerQmlTypes(){
  registerCoreQmlTypes();
  registerStdMsgsQmlTypes();
  registerSensorMsgsQmlTypes();
  registerGeometryMsgsQmlTypes();
  registerDiagnosticMsgsQmlTypes();
  registerRoshipQmlTypes();
}
}
QROS_NS_FOOT
