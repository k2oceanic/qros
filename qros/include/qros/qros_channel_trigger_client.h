#pragma once

/**
 * @file qros_channel_trigger_client.h
 * @brief QML service client for io_interfaces/srv/ChannelTrigger.
 */

#include <io_interfaces/srv/channel_trigger.hpp>
#include <qros/qros_service_client.h>
#include <QString>
#include <QVector>

QROS_NS_HEAD

/**
 * @brief Service client for io_interfaces/srv/ChannelTrigger.
 *
 * Sends a channel-trigger request (1-indexed channel_id, matching the
 * relay-board convention) and exposes the response fields as QML properties.
 *
 * ### QML usage
 * @code{.qml}
 * QRosChannelTriggerClient {
 *     node:        applicationNode
 *     serviceName: "/relay_driver/trigger"
 *     onResponseChanged: console.log("channel trigger:", respSuccess, respMessage)
 * }
 * Button {
 *     onClicked: relayClient.callChannelTrigger(3)  // trigger channel 3
 * }
 * @endcode
 */
class QRosChannelTriggerClient : public QRosServiceClient {
  Q_OBJECT
public:
  /// True if the last response indicated success.
  Q_PROPERTY(bool    respSuccess READ getSuccess  NOTIFY responseChanged)
  /// Human-readable message from the last response.
  Q_PROPERTY(QString respMessage READ getMessage  NOTIFY responseChanged)

public slots:
  /// Returns whether the last channel-trigger response was successful.
  bool getSuccess() {
    return client_.responseBuffer()->success;
  }

  /// Returns the message string from the last channel-trigger response.
  QString getMessage() {
    return QString::fromStdString(client_.responseBuffer()->message);
  }

  /**
   * @brief Sends a channel-trigger request for the specified channel.
   * @param channel_id  1-indexed relay channel number.
   */
  void callChannelTrigger(int channel_id) {
    client_.requestBuffer()->channel_id = channel_id;
    interfacePtr()->callService();
  }

signals:
  /// Emitted when a channel-trigger response is received.
  void responseChanged();

protected:
  QRosServiceClientInterface* interfacePtr() override { return &client_; }
  void onResponseReceived() override { emit responseChanged(); }

private:
  QRosTypedServiceClient<io_interfaces::srv::ChannelTrigger> client_;
};

QROS_NS_FOOT
