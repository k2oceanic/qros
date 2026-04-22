#pragma once

/**
 * @file qros_trigger_service.h
 * @brief QML service client for std_srvs/srv/Trigger.
 */

#include "qros_service_client.h"
#include <std_srvs/srv/trigger.hpp>
#include <QString>

QROS_NS_HEAD

/**
 * @brief Service client for the standard `std_srvs/srv/Trigger` service type.
 *
 * Exposes the response fields `success` and `message` as readable QML
 * properties, updated after each successful callService() call.
 *
 * ### QML usage
 * @code{.qml}
 * QRosTriggerServiceClient {
 *     node:        applicationNode
 *     serviceName: "/arm_system"
 *     onResponseChanged: statusLabel.text = respMessage
 * }
 * @endcode
 */
class QRosTriggerServiceClient : public QRosServiceClient {
  Q_OBJECT
public:
  /// True if the last response indicated success.
  Q_PROPERTY(bool respSuccess READ getSuccess NOTIFY responseChanged)
  /// Human-readable message from the last response.
  Q_PROPERTY(QString respMessage READ getMessage NOTIFY responseChanged)

public slots:
  /// Returns whether the last trigger response was successful.
  bool getSuccess() {
    return client_.responseBuffer()->success;
  }

  /// Returns the message string from the last trigger response.
  QString getMessage() {
    return QString::fromStdString(client_.responseBuffer()->message);
  }

  /// Sends the trigger request.  Blocks until the service responds.
  void callService() {
    auto interface = interfacePtr();
    if (interface) {
      interface->callService();
    } else {
      RCLCPP_ERROR(getRosNode()->get_logger(), "Invalid interface pointer passed to QRosTriggerServiceClient::callService()");
    }
  }

signals:
  /// Emitted when a trigger response is received (success or failure).
  void responseChanged();

protected:
  QRosServiceClientInterface* interfacePtr() override {
    return &client_;
  }

  void onResponseReceived() override {
    emit responseChanged();
  }

private:
  QRosTypedServiceClient<std_srvs::srv::Trigger> client_;
};

QROS_NS_FOOT
