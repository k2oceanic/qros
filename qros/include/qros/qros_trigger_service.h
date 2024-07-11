#pragma once

#include "qros_service_client.h"
#include <std_srvs/srv/trigger.hpp>
#include <QString>

QROS_NS_HEAD

    class QRosTriggerServiceClient : public QRosServiceClient {
  Q_OBJECT
public:
  Q_PROPERTY(bool respSuccess READ getSuccess NOTIFY responseChanged)
  Q_PROPERTY(QString respMessage READ getMessage NOTIFY responseChanged)

public slots:
  bool getSuccess() {
    return client_.responseBuffer()->success;
  }

  QString getMessage() {
    return QString::fromStdString(client_.responseBuffer()->message);
  }

  void callService() {
    auto interface = interfacePtr();
    if (interface) {
      interface->callService();
    } else {
      RCLCPP_ERROR(getRosNode()->get_logger(), "Invalid interface pointer passed to QRosTriggerServiceClient::callService()");
    }
  }

signals:
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
