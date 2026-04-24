#pragma once

/**
 * @file qros_raw_analog_array.h
 * @brief Subscriber for io_interfaces/msg/RawAnalogArray.
 */

#include "qros_subscriber.h"
#include "qros_publisher.h"
#include <io_interfaces/msg/raw_analog_array.hpp>

#include <QString>
#include <QVector>
#include <QDateTime>

QROS_NS_HEAD

/**
 * @brief Subscribes to `io_interfaces/RawAnalogArray` messages.
 *
 * Exposes all per-channel analog data from the IO board.  In addition to
 * the bulk array properties, the `analogChanged` signal fires once per
 * channel per message, making it easy to route individual channels to
 * separate QML components without iterating the arrays.
 *
 * ### QML usage — bulk binding
 * @code{.qml}
 * QRosRawAnalogSubscriber {
 *     node:  applicationNode
 *     topic: "/io_board/analog"
 *     onAnalogsChanged: {
 *         for (var i = 0; i < channelIds.length; i++)
 *             channels[channelIds[i]].value = scaledValues[i]
 *     }
 * }
 * @endcode
 *
 * ### QML usage — per-channel signal
 * @code{.qml}
 * QRosRawAnalogSubscriber {
 *     node:  applicationNode
 *     topic: "/io_board/analog"
 *     onAnalogChanged: (channelId, scale, prop, scaled, type) => {
 *         if (channelId === 3) batteryDisplay.value = scaled
 *     }
 * }
 * @endcode
 */
class QRosRawAnalogSubscriber : public QRosSubscriber {
  Q_OBJECT
public:
  /// Channel IDs present in the last received message.
  Q_PROPERTY(QVector<int> channelIds READ getChannelIds NOTIFY analogsChanged)
  /// Scale factors for each channel (engineering-unit multiplier).
  Q_PROPERTY(QVector<double> scales READ getScales NOTIFY analogsChanged)
  /// Proportional (0–1) ADC values for each channel.
  Q_PROPERTY(QVector<double> proportionalValues READ getProportionalValues NOTIFY analogsChanged)
  /// Scaled values (proportionalValue × scale) for each channel.
  Q_PROPERTY(QVector<double> scaledValues READ getScaledValues NOTIFY analogsChanged)
  /// Channel type bytes for each channel.
  Q_PROPERTY(QVector<uint8_t> types READ getTypes NOTIFY analogsChanged)
  /// Frame ID from the last received message header.
  Q_PROPERTY(QString frameId READ getFrameId NOTIFY analogsChanged)

public slots:
  QVector<int> getChannelIds() {
    QVector<int> ids;
    for (auto &analog : subscriber_.msgBuffer().analogs) {
      ids.push_back(analog.channel_id);
    }
    return ids;
  }

  QVector<double> getScales() {
    QVector<double> scales;
    for (auto &analog : subscriber_.msgBuffer().analogs) {
      scales.push_back(analog.scale);
    }
    return scales;
  }

  QVector<double> getProportionalValues() {
    QVector<double> values;
    for (auto &analog : subscriber_.msgBuffer().analogs) {
      values.push_back(analog.proportional_value);
    }
    return values;
  }

  QVector<double> getScaledValues() {
    QVector<double> values;
    for (auto &analog : subscriber_.msgBuffer().analogs) {
      values.push_back(analog.proportional_value * analog.scale);
    }
    return values;
  }

  QVector<uint8_t> getTypes() {
    QVector<uint8_t> types;
    for (auto &analog : subscriber_.msgBuffer().analogs) {
      types.push_back(analog.type);
    }
    return types;
  }

  QString getFrameId() {
    return QString::fromStdString(subscriber_.msgBuffer().header.frame_id);
  }

signals:
  /// Emitted once per message after all per-channel signals.
  void analogsChanged();
  /**
   * @brief Emitted once per channel per received message.
   * @param channelId        1-based hardware channel number.
   * @param scale            Engineering-unit scale factor.
   * @param proportionalValue Raw ADC value in the 0–1 range.
   * @param scaledValue      proportionalValue × scale.
   * @param type             Channel type byte.
   */
  void analogChanged(int channelId, double scale, double proportionalValue, double scaledValue, int type);

protected:
  void onMsgReceived() override {
    for (auto &analog : subscriber_.msgBuffer().analogs) {
      emit analogChanged(analog.channel_id,
                         analog.scale,
                         analog.proportional_value,
                         analog.scale * analog.proportional_value,
                         analog.type);
    }
    emit analogsChanged();
  }

private:
  QRosSubscriberInterface* interfacePtr() { return &subscriber_; }
  QRosTypedSubscriber<io_interfaces::msg::RawAnalogArray> subscriber_;
};

QROS_NS_FOOT
