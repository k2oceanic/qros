#pragma once

/**
 * @file qros_raw_digital.h
 * @brief Publisher and subscriber for io_interfaces/msg/RawDigitalArray.
 */

#include <io_interfaces/msg/raw_digital_array.hpp>
#include <qros/qros_subscriber.h>
#include <qros/qros_publisher.h>
#include <QString>
#include <QVector>

QROS_NS_HEAD

/**
 * @brief Subscribes to `io_interfaces/RawDigitalArray` messages.
 *
 * Works with any number of channels — not limited to 6.
 * Channels are 1-indexed (channel_id 1 → index 0 in states/channelIds vectors).
 * The `states` vector is sized to `max(channel_id)` with gaps filled to false.
 *
 * In addition to the bulk `digitalsChanged` signal, `digitalChanged` fires
 * once per channel per message for easy per-channel routing.
 *
 * ### QML usage — bulk
 * @code{.qml}
 * QRosRawDigitalArraySubscriber {
 *     node:  applicationNode
 *     topic: "/io_board/digital"
 *     onDigitalsChanged: relay3Indicator.active = states[2]
 * }
 * @endcode
 *
 * ### QML usage — per-channel signal
 * @code{.qml}
 * QRosRawDigitalArraySubscriber {
 *     node:  applicationNode
 *     topic: "/io_board/digital"
 *     onDigitalChanged: (channelId, state) => {
 *         if (channelId === 4) pumpIndicator.active = state
 *     }
 * }
 * @endcode
 */
class QRosRawDigitalArraySubscriber : public QRosSubscriber {
  Q_OBJECT
public:
  /// Channel IDs present in the last received message.
  Q_PROPERTY(QVector<int>  channelIds READ getChannelIds NOTIFY digitalsChanged)
  /// Boolean states indexed from 0 (channel_id 1 = index 0); gaps filled with false.
  Q_PROPERTY(QVector<bool> states     READ getStates     NOTIFY digitalsChanged)
  /// Frame ID from the last received message header.
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
  /// Emitted once per message after all per-channel signals.
  void digitalsChanged();
  /**
   * @brief Emitted once per channel per received message.
   * @param channelId  1-based hardware channel number.
   * @param state      Current state of the channel.
   */
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
 * @brief Publishes `io_interfaces/RawDigitalArray` messages.
 *
 * Use setChannel() to command a single relay, or setAllChannels() to send
 * the full state of all channels in one message.
 *
 * ### QML usage
 * @code{.qml}
 * QRosRawDigitalArrayPublisher {
 *     id:    digitalPub
 *     node:  applicationNode
 *     topic: "/io_board/digital_cmd"
 * }
 * Switch {
 *     onCheckedChanged: {
 *         digitalPub.setChannel(3, checked)
 *         digitalPub.publish()
 *     }
 * }
 * @endcode
 */
class QRosRawDigitalArrayPublisher : public QRosPublisher {
  Q_OBJECT
public:
public slots:
  /**
   * @brief Stages a single-channel update (replaces the current message buffer).
   * @param channel_id  1-based channel number.
   * @param state       Desired state.
   */
  void setChannel(int channel_id, bool state) {
    publisher_.msgBuffer().digitals.clear();
    io_interfaces::msg::RawDigital d;
    d.channel_id = channel_id;
    d.state = state;
    publisher_.msgBuffer().digitals.push_back(d);
  }

  /**
   * @brief Stages all channels (replaces the current message buffer).
   * @param states  Boolean states starting from channel 1.
   */
  void setAllChannels(QVector<bool> states) {
    publisher_.msgBuffer().digitals.clear();
    for (int i = 0; i < states.size(); ++i) {
      io_interfaces::msg::RawDigital d;
      d.channel_id = i + 1;
      d.state = states[i];
      publisher_.msgBuffer().digitals.push_back(d);
    }
  }

  /// Publishes the current message buffer.
  void publish() { publisher_.publish(); }

protected:
  QRosPublisherInterface* interfacePtr() override { return &publisher_; }
  QRosTypedPublisher<io_interfaces::msg::RawDigitalArray> publisher_;
};

QROS_NS_FOOT
