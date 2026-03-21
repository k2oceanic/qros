#pragma once

#include <io_interfaces/msg/raw_digital_array.hpp>
#include <io_interfaces/srv/trip_reset.hpp>
#include <qros/qros_subscriber.h>
#include <qros/qros_publisher.h>
#include <qros/qros_service_client.h>
#include <QVector>

QROS_NS_HEAD

/**
 * @brief Subscribes to a RawDigitalArray topic and exposes channel states as a QVector<bool>.
 *
 * The states vector is always 6 elements, indexed by channel_id.
 * Any channel_id not present in the message defaults to false.
 */
class QRosRawDigitalArraySubscriber : public QRosSubscriber {
  Q_OBJECT
public:
  Q_PROPERTY(QVector<bool> states READ getStates NOTIFY statesChanged)

public slots:
  QVector<bool> getStates() {
    QVector<bool> result(6, false);
    for (const auto& d : subscriber_.msgBuffer().digitals) {
      if (d.channel_id >= 0 && d.channel_id < 6) {
        result[d.channel_id] = d.state;
      }
    }
    return result;
  }

signals:
  void statesChanged();

protected:
  void onMsgReceived() override { emit statesChanged(); }

private:
  QRosSubscriberInterface* interfacePtr() override { return &subscriber_; }
  QRosTypedSubscriber<io_interfaces::msg::RawDigitalArray> subscriber_;
};


/**
 * @brief Publishes a RawDigitalArray topic.
 *
 * setChannel() clears the array and sets a single element, ensuring a clean
 * single-channel publish on each call. Use setAllChannels() to batch-set all channels.
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
    for (int i = 0; i < std::min(6, (int)states.size()); ++i) {
      io_interfaces::msg::RawDigital d;
      d.channel_id = i;
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
 * Follows the QRosTriggerServiceClient pattern from qros_trigger_service.h.
 */
class QRosTripResetClient : public QRosServiceClient {
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
