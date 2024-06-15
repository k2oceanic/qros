#pragma once

#include "qros_subscriber.h"
#include "qros_publisher.h"
#include <io_interfaces/msg/raw_analog_array.hpp>

#include <QString>
#include <QVector>
#include <QDateTime>

QROS_NS_HEAD

//     class QRosRawAnalogPublisher : public QRosPublisher {
//   Q_OBJECT
// public:
//   Q_PROPERTY(QVector<int> channelIds READ getChannelIds WRITE setChannelIds NOTIFY analogsChanged)
//   Q_PROPERTY(QVector<float> scales READ getScales WRITE setScales NOTIFY analogsChanged)
//   Q_PROPERTY(QVector<float> proportionalValues READ getProportionalValues WRITE setProportionalValues NOTIFY analogsChanged)
//   Q_PROPERTY(QString frameId READ getFrameId WRITE setFrameId NOTIFY analogsChanged)

// public slots:
//   QVector<int> getChannelIds() {
//     QVector<int> ids;
//     for (auto &analog : publisher_.msgBuffer()) {
//       ids.push_back(analog.channel_id);
//     }
//     return ids;
//   }

//   void setChannelIds(QVector<int> ids) {
//     for (int i = 0; i < ids.size(); ++i) {
//       publisher_.msgBuffer().analogs[i].channel_id = ids[i];
//     }
//     emit analogsChanged();
//   }

//   QVector<float> getScales() {
//     QVector<float> scales;
//     for (auto &analog : publisher_.msgBuffer().analogs) {
//       scales.push_back(analog.scale);
//     }
//     return scales;
//   }

//   void setScales(QVector<float> scales) {
//     for (int i = 0; i < scales.size(); ++i) {
//       publisher_.msgBuffer().analogs[i].scale = scales[i];
//     }
//     emit analogsChanged();
//   }

//   QVector<float> getProportionalValues() {
//     QVector<float> values;
//     for (auto &analog : publisher_.msgBuffer().analogs) {
//       values.push_back(analog.proportional_value);
//     }
//     return values;
//   }

//   void setProportionalValues(QVector<float> values) {
//     for (int i = 0; i < values.size(); ++i) {
//       publisher_.msgBuffer().analogs[i].proportional_value = values[i];
//     }
//     emit analogsChanged();
//   }

//   QString getFrameId() {
//     return QString::fromStdString(publisher_.msgBuffer().header.frame_id);
//   }

//   void setFrameId(QString frameId) {
//     publisher_.msgBuffer().header.frame_id = frameId.toStdString();
//     emit analogsChanged();
//   }

// signals:
//   void analogsChanged();

// protected:
//   QRosPublisherInterface* interfacePtr() { return &publisher_; }
//   QRosTypedPublisher<io_interfaces::msg::RawAnalogArray> publisher_;
// };

class QRosRawAnalogSubscriber : public QRosSubscriber {
  Q_OBJECT
public:
  Q_PROPERTY(QVector<int> channelIds READ getChannelIds NOTIFY analogsChanged)
  Q_PROPERTY(QVector<double> scales READ getScales NOTIFY analogsChanged)
  Q_PROPERTY(QVector<double> proportionalValues READ getProportionalValues NOTIFY analogsChanged)
  Q_PROPERTY(QVector<double> scaledValues READ getScaledValues NOTIFY analogsChanged)
  Q_PROPERTY(QVector<uint8_t> types READ getTypes NOTIFY analogsChanged)
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
  void analogsChanged();
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
