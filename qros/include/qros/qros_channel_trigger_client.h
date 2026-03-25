#pragma once

#include <io_interfaces/srv/channel_trigger.hpp>
#include <qros/qros_service_client.h>
#include <QString>
#include <QVector>

QROS_NS_HEAD

/**
 * @brief Service client for io_interfaces/srv/ChannelTrigger.
 *
 * Call callChannelTrigger(channel_id) to send a channel trigger request.
 * channel_id is 1-indexed to match the relay board convention.
 */
class QRosChannelTriggerClient : public QRosServiceClient {
  Q_OBJECT
public:
  Q_PROPERTY(bool    respSuccess READ getSuccess  NOTIFY responseChanged)
  Q_PROPERTY(QString respMessage READ getMessage  NOTIFY responseChanged)

public slots:
  bool getSuccess() {
    return client_.responseBuffer()->success;
  }

  QString getMessage() {
    return QString::fromStdString(client_.responseBuffer()->message);
  }

  void callChannelTrigger(int channel_id) {
    client_.requestBuffer()->channel_id = channel_id;
    interfacePtr()->callService();
  }

signals:
  void responseChanged();

protected:
  QRosServiceClientInterface* interfacePtr() override { return &client_; }
  void onResponseReceived() override { emit responseChanged(); }

private:
  QRosTypedServiceClient<io_interfaces::srv::ChannelTrigger> client_;
};

QROS_NS_FOOT
