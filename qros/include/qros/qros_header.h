#pragma once

#include "qros_publisher.h"
#include "qros_subscriber.h"
#include <QObject>
#include <QDateTime>
#include <QString>
#include <std_msgs/msg/header.hpp>
#include "qros_message.h"

QROS_NS_HEAD

class QRosHeader : public QRosMessageType<std_msgs::msg::Header> {
  Q_OBJECT
  Q_PROPERTY(QDateTime stamp READ getStamp WRITE setStamp NOTIFY stampChanged)
  Q_PROPERTY(QString frameId READ getFrameId WRITE setFrameId NOTIFY frameIdChanged)

public:
  using QRosMessageType<std_msgs::msg::Header>::QRosMessageType;
  QDateTime getStamp() const {
    uint32_t sec = ros_msg_->stamp.sec;
    uint32_t nsec = ros_msg_->stamp.nanosec;
    QDateTime dateTime = QDateTime::fromSecsSinceEpoch(sec);
    dateTime = dateTime.addMSecs(nsec / 1e6);
    return dateTime;
  }

  void setStamp(const QDateTime &dateTime) {
    ros_msg_->stamp.sec = dateTime.toSecsSinceEpoch();
    ros_msg_->stamp.nanosec = (dateTime.toMSecsSinceEpoch() % 1000) * 1e6;
    emit stampChanged();
  }

  QString getFrameId() const {
    return QString::fromStdString(ros_msg_->frame_id);
  }

  void setFrameId(const QString &frameId) {
    ros_msg_->frame_id = frameId.toStdString();
    emit frameIdChanged();
  }

signals:
  void stampChanged();
  void frameIdChanged();
};

class QRosHeaderPublisher : public QRosPublisher{
  Q_OBJECT
public:
  Q_PROPERTY(QRosHeader * message READ getMessage WRITE setMessage NOTIFY messageChanged)
public slots:
  QRosHeader * getMessage(){
    return msg_;
  }
  void setMessage(QRosHeader * msg){
    publisher_.setMsgBuffer(msg);
    emit messageChanged();
  }
signals:
  void messageChanged();

protected:
  QRosPublisherInterface * interfacePtr(){return &publisher_;}
  QRosTypedPublisher<QRosHeader> publisher_;
  QRosHeader * msg_;
};

QROS_NS_FOOT


