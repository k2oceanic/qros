#pragma once

#include "qros_publisher.h"
#include "qros_subscriber.h"
#include "io_interfaces/msg/raw_packet.hpp" 
#include <QString>
#include <QByteArray>

QROS_NS_HEAD

class QRosRawPacketPublisher : public QRosPublisher{
  Q_OBJECT
public:
  Q_PROPERTY(QByteArray data READ getData WRITE setData NOTIFY dataChanged)
public slots:
  QByteArray getData(){
    return QByteArray(reinterpret_cast<const char*>(publisher_.msgBuffer().data.data()), publisher_.msgBuffer().data.size());
  }
  void setData(const QByteArray& data){
    publisher_.msgBuffer().data = std::vector<uint8_t>(data.begin(), data.end());
    emit dataChanged();
  }
signals:
  void dataChanged();

protected:
  QRosPublisherInterface * interfacePtr(){return &publisher_;}
  QRosTypedPublisher<io_interfaces::msg::RawPacket> publisher_;  
};


class QRosRawPacketSubscriber : public QRosSubscriber{
  Q_OBJECT
public:
  Q_PROPERTY(QString dataAsAscii READ getDataAsAscii NOTIFY dataChanged)
  Q_PROPERTY(QString dataAsHex READ getDataAsHex NOTIFY dataChanged)
public slots:
  QString getDataAsAscii() {
    return QString::fromStdString(std::string(subscriber_.msgBuffer().data.begin(), subscriber_.msgBuffer().data.end()));
  }

  QString getDataAsHex() {
    const auto& data = subscriber_.msgBuffer().data;
    QString hexString;
    for (uint8_t byte : data) {
      hexString.append(QString::asprintf("%02X ", byte));
    }
    return hexString.trimmed();
  }

signals:
  void dataChanged();

protected:
  void onMsgReceived() override{
    emit dataChanged();
  }

private:
  QRosSubscriberInterface* interfacePtr(){return &subscriber_;}
  QRosTypedSubscriber<io_interfaces::msg::RawPacket> subscriber_;  
};

QROS_NS_FOOT
