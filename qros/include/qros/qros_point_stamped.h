#pragma once

#include "qros_subscriber.h"
#include "qros_publisher.h"
#include <geometry_msgs/msg/point_stamped.hpp>
#include <QString>
#include <QVector3D>
#include <QDateTime>

QROS_NS_HEAD

    class QRosPointStampedPublisher : public QRosPublisher{
  Q_OBJECT
public:
  Q_PROPERTY(QVector3D position READ getPosition WRITE setPosition NOTIFY pointChanged)
  Q_PROPERTY(QString frameId READ getFrameId WRITE setFrameId NOTIFY pointChanged)
  Q_PROPERTY(QDateTime timestamp READ getTimestamp WRITE setTimestamp NOTIFY pointChanged)

public slots:
  QVector3D getPosition(){
    auto position = publisher_.msgBuffer().point;
    return QVector3D(position.x, position.y, position.z);
  }
  void setPosition(QVector3D pos){
    publisher_.msgBuffer().point.x = pos.x();
    publisher_.msgBuffer().point.y = pos.y();
    publisher_.msgBuffer().point.z = pos.z();
    emit pointChanged();
  }

  QString getFrameId() {
    return QString::fromStdString(publisher_.msgBuffer().header.frame_id);
  }

  void setFrameId(QString frameId) {
    publisher_.msgBuffer().header.frame_id = frameId.toStdString();
    emit pointChanged();
  }

  QDateTime getTimestamp() {
    qint64 totalMSecs = static_cast<qint64>(publisher_.msgBuffer().header.stamp.sec) * 1000LL +
                        static_cast<qint64>(publisher_.msgBuffer().header.stamp.nanosec) / 1000000LL;
    return QDateTime::fromMSecsSinceEpoch(totalMSecs);
  }

  void setTimestamp(QDateTime timestamp) {
    publisher_.msgBuffer().header.stamp.sec = timestamp.toSecsSinceEpoch();
    publisher_.msgBuffer().header.stamp.nanosec = (timestamp.toMSecsSinceEpoch() % 1000) * 1000000;
    emit pointChanged();
  }

signals:
  void pointChanged();

protected:
  QRosPublisherInterface* interfacePtr(){return &publisher_;}
  QRosTypedPublisher<geometry_msgs::msg::PointStamped> publisher_;
};



class QRosPointStampedSubscriber : public QRosSubscriber{
  Q_OBJECT
public:
  Q_PROPERTY(QVector3D position READ getPosition NOTIFY pointChanged)
  Q_PROPERTY(QString frameId READ getFrameId NOTIFY pointChanged)
  Q_PROPERTY(QDateTime timestamp READ getTimestamp NOTIFY pointChanged)

public slots:
  QVector3D getPosition() {
    auto position = subscriber_.msgBuffer().point;
    return QVector3D(position.x, position.y, position.z);
  }

  QString getFrameId() {
    return QString::fromStdString(subscriber_.msgBuffer().header.frame_id);
  }

  QDateTime getTimestamp() {
    qint64 totalMSecs = static_cast<qint64>(subscriber_.msgBuffer().header.stamp.sec) * 1000LL +
                        static_cast<qint64>(subscriber_.msgBuffer().header.stamp.nanosec) / 1000000LL;
    return QDateTime::fromMSecsSinceEpoch(totalMSecs);
  }

signals:
  void pointChanged();

protected:
  void onMsgReceived() override{
    emit pointChanged();
  }

private:
  QRosSubscriberInterface* interfacePtr(){return &subscriber_;}
  QRosTypedSubscriber<geometry_msgs::msg::PointStamped> subscriber_;
};

QROS_NS_FOOT
