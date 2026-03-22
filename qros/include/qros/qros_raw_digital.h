#pragma once

#include <io_interfaces/msg/raw_digital_array.hpp>
#include <io_interfaces/srv/trip_reset.hpp>
#include <qros/qros_subscriber.h>
#include <qros/qros_publisher.h>
#include <qros/qros_service_client.h>
#include <QString>
#include <QVector>

QROS_NS_HEAD

/**
 * @brief Subscribes to a RawDigitalArray topic and exposes channel data as QML properties.
 *
 * Works with any number of channels — not limited to 6.
 * Channels are 1-indexed (channel_id 1 = index 0 in states/channelIds vectors).
 * states[i] corresponds to channelIds[i].
 */
class QRosRawDigitalArraySubscriber : public QRosSubscriber {
  Q_OBJECT
public:
  Q_PROPERTY(QVector<int>  channelIds READ getChannelIds NOTIFY digitalsChanged)
  Q_PROPERTY(QVector<bool> states     READ getStates     NOTIFY digitalsChanged)
  Q_PROPERTY(QString       frameId    READ getFrameId    NOTIFY digitalsChanged)

public slots:
  QVector<int> getChannelIds() {
    QVector<int> ids;
    for (const auto& d : subscriber_.msgBuffer().digitals) {
      ids.push_back(d.channel_id);
    }
    return ids;
  }

  QVector<bool> getStates() {
    QVector<bool> result;
    for (const auto& d : subscriber_.msgBuffer().digitals) {
      int idx = d.channel_id - 1;
      if (idx >= result.size()) result.resize(idx + 1, false);
      result[idx] = d.state;
    }
    return result;
  }

  QString getFrameId() {
    return QString::fromStdString(subscriber_.msgBuffer().header.frame_id);
  }

signals:
  void digitalsChanged();
  void digitalChanged(int channelId, bool state);

protected:
  void onMsgReceived() override {
    for (const auto& d : subscriber_.msgBuffer().digitals) {
      emit digitalChanged(d.channel_id, d.state);
    }
    emit digitalsChanged();
  }

private:
  QRosSubscriberInterface* interfacePtr() override { return &subscriber_; }
  QRosTypedSubscriber<io_interfaces::msg::RawDigitalArray> subscriber_;
};


/**
 * @brief Publishes a RawDigitalArray topic.
 *
 * setChannel() sets a single channel by 1-based channel_id.
 * setAllChannels() sets all channels starting from channel 1, sized to the input vector.
 */
class QRosRawDigitalArrayPublisher : public QRosPublisher {
  Q_OBJECT
public:
public slots:
  void setChannel(int channel_id, bool state) {
    publisher_.msgBuffer().digitals.clear();
    io_interfaces::msg::RawDigital d;
    d.channel_id = channel_id;
    d.state = state;
    publisher_.msgBuffer().digitals.push_back(d);
  }

  void setAllChannels(QVector<bool> states) {
    publisher_.msgBuffer().digitals.clear();
    for (int i = 0; i < states.size(); ++i) {
      io_interfaces::msg::RawDigital d;
      d.channel_id = i + 1;
      d.state = states[i];
      publisher_.msgBuffer().digitals.push_back(d);
    }
  }

  void publish() { publisher_.publish(); }

protected:
  QRosPublisherInterface* interfacePtr() override { return &publisher_; }
  QRosTypedPublisher<io_interfaces::msg::RawDigitalArray> publisher_;
};


/**
 * @brief Service client for io_interfaces/srv/TripReset.
 *
 * Call callTripReset(channel_id) to send a trip reset request.
 * channel_id is 1-indexed to match the relay board convention.
 */
class QRosTripResetClient : public QRosServiceClient {
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

  void callTripReset(int channel_id) {
    client_.requestBuffer()->channel_id = channel_id;
    interfacePtr()->callService();
  }

signals:
  void responseChanged();

protected:
  QRosServiceClientInterface* interfacePtr() override { return &client_; }
  void onResponseReceived() override { emit responseChanged(); }

private:
  QRosTypedServiceClient<io_interfaces::srv::TripReset> client_;
};

QROS_NS_FOOT
