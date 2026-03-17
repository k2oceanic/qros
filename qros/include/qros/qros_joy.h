#pragma once

#include "qros_subscriber.h"
#include "qros_publisher.h"
#include <sensor_msgs/msg/joy.hpp>
#include <QString>
#include <QVector>
#include <QDateTime>

QROS_NS_HEAD

    class QRosJoyPublisher : public QRosPublisher {
  Q_OBJECT
public:
  Q_PROPERTY(QVector<float> axes READ getAxes WRITE setAxes NOTIFY joyChanged)
  Q_PROPERTY(QVector<int> buttons READ getButtons WRITE setButtons NOTIFY joyChanged)
  Q_PROPERTY(QString frameId READ getFrameId WRITE setFrameId NOTIFY joyChanged)
  Q_PROPERTY(QDateTime timestamp READ getTimestamp WRITE setTimestamp NOTIFY joyChanged)

public slots:
  void resizeAxes(int size) {
    publisher_.msgBuffer().axes.resize(size);
  }

  QVector<float> getAxes() {
    return QVector<float>(publisher_.msgBuffer().axes.begin(), publisher_.msgBuffer().axes.end());
  }

  void setAxes(const QVector<float>& axes) {
    publisher_.msgBuffer().axes = std::vector<float>(axes.begin(), axes.end());
    emit joyChanged();
  }

  void setAxis(int index, float value) {
    if (index < publisher_.msgBuffer().axes.size()) {
      publisher_.msgBuffer().axes[index] = value;
    }
  }

  void resizeButtons(int size) {
    publisher_.msgBuffer().buttons.resize(size);
  }

  QVector<int> getButtons() {
    return QVector<int>(publisher_.msgBuffer().buttons.begin(), publisher_.msgBuffer().buttons.end());
  }

  void setButtons(const QVector<int>& buttons) {
    publisher_.msgBuffer().buttons = std::vector<int>(buttons.begin(), buttons.end());
    emit joyChanged();
  }

  void setButton(int index, int value) {
    if (index < publisher_.msgBuffer().buttons.size()) {
      publisher_.msgBuffer().buttons[index] = value;
    }
  }

  QString getFrameId() {
    return QString::fromStdString(publisher_.msgBuffer().header.frame_id);
  }

  void setFrameId(const QString& frameId) {
    publisher_.msgBuffer().header.frame_id = frameId.toStdString();
    emit joyChanged();
  }

  QDateTime getTimestamp() {
    qint64 totalMSecs = static_cast<qint64>(publisher_.msgBuffer().header.stamp.sec) * 1000LL +
                        static_cast<qint64>(publisher_.msgBuffer().header.stamp.nanosec) / 1000000LL;
    return QDateTime::fromMSecsSinceEpoch(totalMSecs);
  }

  void setTimestamp(const QDateTime& timestamp) {
    publisher_.msgBuffer().header.stamp.sec = timestamp.toSecsSinceEpoch();
    publisher_.msgBuffer().header.stamp.nanosec = (timestamp.toMSecsSinceEpoch() % 1000) * 1000000;
    emit joyChanged();
  }

signals:
  void joyChanged();

protected:
  QRosPublisherInterface* interfacePtr() { return &publisher_; }
  QRosTypedPublisher<sensor_msgs::msg::Joy> publisher_;
};

class QRosJoySubscriber : public QRosSubscriber {
  Q_OBJECT
public:
  Q_PROPERTY(QVector<float> axes READ getAxes NOTIFY joyChanged)
  Q_PROPERTY(QVector<int> buttons READ getButtons NOTIFY joyChanged)
  Q_PROPERTY(QString frameId READ getFrameId NOTIFY joyChanged)
  Q_PROPERTY(QDateTime timestamp READ getTimestamp NOTIFY joyChanged)

public slots:
  QVector<float> getAxes() {
    return QVector<float>(subscriber_.msgBuffer().axes.begin(), subscriber_.msgBuffer().axes.end());
  }

  QVector<int> getButtons() {
    return QVector<int>(subscriber_.msgBuffer().buttons.begin(), subscriber_.msgBuffer().buttons.end());
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
  void joyChanged();
  void pressed(int buttonIndex);   // Signal for when a button is pressed
  void released(int buttonIndex); // Signal for when a button is released

protected:
  void onMsgReceived() override {
    compareAndEmitButtonPresses();
    emit joyChanged();
  }

private:
  QVector<int> lastButtonStates;

  void compareAndEmitButtonPresses() {
    auto currentButtons = getButtons();

    if (lastButtonStates.isEmpty()) {
      lastButtonStates = QVector<int>(currentButtons.size(), 0);
    }

    int longestArraySize = std::max(currentButtons.size(), lastButtonStates.size());
    currentButtons.resize(longestArraySize);
    lastButtonStates.resize(longestArraySize);

    for (int i = 0; i < longestArraySize; ++i) {
      if (currentButtons[i] > 0 && lastButtonStates[i] == 0) {
        emit pressed(i);
      } else if (currentButtons[i] == 0 && lastButtonStates[i] > 0) {
        emit released(i);
      }
    }

    lastButtonStates = currentButtons;
  }

  QRosSubscriberInterface* interfacePtr() { return &subscriber_; }
  QRosTypedSubscriber<sensor_msgs::msg::Joy> subscriber_;
};

QROS_NS_FOOT
