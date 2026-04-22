#pragma once

/**
 * @file qros_string_subscriber.h
 * @brief Subscriber for std_msgs/msg/String.
 */

#include "qros_subscriber.h"
#include <std_msgs/msg/string.hpp>
#include <QString>

QROS_NS_HEAD

/**
 * @brief Subscribes to `std_msgs/String` messages.
 *
 * ### QML usage
 * @code{.qml}
 * QRosStringSubscriber {
 *     node:  applicationNode
 *     topic: "/status_label"
 *     onDataChanged: statusText.text = data
 * }
 * @endcode
 */
class QRosStringSubscriber : public QRosSubscriber{
  Q_OBJECT
public:
  /// The last received string value.
  Q_PROPERTY(QString data READ getData NOTIFY dataChanged)
public slots:
  QString getData() {return QString::fromStdString(subscriber_.msgBuffer().data);}

signals:
  void dataChanged();

protected:
  void onMsgReceived() override{
    emit dataChanged();
  }

private:
  QRosSubscriberInterface* interfacePtr(){return &subscriber_;}
  QRosTypedSubscriber<std_msgs::msg::String> subscriber_;
};

QROS_NS_FOOT
