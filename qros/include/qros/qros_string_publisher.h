#pragma once

/**
 * @file qros_string_publisher.h
 * @brief Publisher for std_msgs/msg/String.
 */

#include "qros_publisher.h"
#include <std_msgs/msg/string.hpp>
#include <QString>

QROS_NS_HEAD

/**
 * @brief Publishes `std_msgs/String` messages.
 *
 * ### QML usage
 * @code{.qml}
 * QRosStringPublisher {
 *     node:  applicationNode
 *     topic: "/status_label"
 *     data:  statusTextField.text
 * }
 * @endcode
 */
class QRosStringPublisher : public QRosPublisher{
  Q_OBJECT
public:
  /// The string value to publish.
  Q_PROPERTY(QString data READ getData WRITE setData NOTIFY dataChanged)
public slots:
  QString getData(){
    return QString::fromStdString(publisher_.msgBuffer().data);
  }
  void setData(QString topic){
    publisher_.msgBuffer().data = topic.toStdString();
    emit dataChanged();
  }
signals:
  void dataChanged();

protected:
  QRosPublisherInterface * interfacePtr(){return &publisher_;}
  QRosTypedPublisher<std_msgs::msg::String> publisher_;
};

QROS_NS_FOOT
