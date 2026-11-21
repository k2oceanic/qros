#include "qros_register.hpp"

#include "qros/qros_diagnostic_status.h"
#include "qros/qros_diagnostic_array_subscriber.h"
#include "qros/qros_diagnostics_updater.h"

QROS_NS_HEAD
namespace qros {
void registerDiagnosticMsgsQmlTypes(){
  REGISTER_QML_TYPE(QRosDiagnosticStatusSubscriber)
  REGISTER_QML_TYPE(QRosDiagnosticStatusPublisher)
  REGISTER_QML_TYPE(QRosDiagnosticArraySubscriber)
  REGISTER_QML_TYPE(QRosDiagnosticTask)
  REGISTER_QML_TYPE(QRosDiagnosticsUpdater)
}
}
QROS_NS_FOOT
