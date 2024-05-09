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
  void resizeAxes(int size){
    publisher_.msgBuffer().axes.resize(size);
  }
  QVector<float> getAxes() {
    auto axes = QVector<float>::fromStdVector(publisher_.msgBuffer().axes);
    return axes;
  }
  void setAxes(QVector<float> axes) {
    publisher_.msgBuffer().axes = axes.toStdVector();
    emit joyChanged();
  }
  void setAxis(int index, float value){
    if(index < publisher_.msgBuffer().axes.size()){
      publisher_.msgBuffer().axes[index] = value;
    }
  }

  void resizeButtons(int size){
    publisher_.msgBuffer().buttons.resize(size);
  }
  QVector<int> getButtons() {
    auto buttons = QVector<int>::fromStdVector(publisher_.msgBuffer().buttons);
    return buttons;
  }
  void setButtons(QVector<int> buttons) {
    publisher_.msgBuffer().buttons = buttons.toStdVector();
    emit joyChanged();
  }
  void setButton(int index, int value){
    if(index < publisher_.msgBuffer().buttons.size()){
      publisher_.msgBuffer().buttons[index] = value;
    }
  }

  QString getFrameId() {
    return QString::fromStdString(publisher_.msgBuffer().header.frame_id);
  }

  void setFrameId(QString frameId) {
    publisher_.msgBuffer().header.frame_id = frameId.toStdString();
    emit joyChanged();
  }

  QDateTime getTimestamp() {
    qint64 totalMSecs = static_cast<qint64>(publisher_.msgBuffer().header.stamp.sec) * 1000LL +
                        static_cast<qint64>(publisher_.msgBuffer().header.stamp.nanosec) / 1000000LL;
    return QDateTime::fromMSecsSinceEpoch(totalMSecs);
  }

  void setTimestamp(QDateTime timestamp) {
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
    auto axes = QVector<float>::fromStdVector(subscriber_.msgBuffer().axes);
    return axes;
  }

  QVector<int> getButtons() {
    auto buttons = QVector<int>::fromStdVector(subscriber_.msgBuffer().buttons);
    return buttons;
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
  void pressed(int buttonIndex);  // Signal for when a button is pressed
  void released(int buttonIndex); // Signal for when a button is released

protected:
  void onMsgReceived() override {
    compareAndEmitButtonPresses();
    emit joyChanged();
  }

private:
  QVector<int> lastButtonStates;

  // Compare current button states to last known states and emit onPressed or onReleased as needed
  void compareAndEmitButtonPresses() {
    auto currentButtons = getButtons();
    // Initialize lastButtonStates with zeros on the first message
    if (lastButtonStates.isEmpty()) {
      lastButtonStates = QVector<int>(currentButtons.size(), 0);
    }

    // Check for the longest array to avoid index out of range
    int longestArraySize = std::max(currentButtons.size(), lastButtonStates.size());

    // Resize arrays to match the longest one, filling new elements with 0
    currentButtons.resize(longestArraySize);
    lastButtonStates.resize(longestArraySize);

    for (int i = 0; i < longestArraySize; ++i) {
      // Emit onPressed if the current state is pressed (greater than 0) and was not pressed before
      if (currentButtons[i] > 0 && lastButtonStates[i] == 0) {
        emit pressed(i);
      }
      // Emit onReleased if the current state is not pressed and was pressed before
      else if (currentButtons[i] == 0 && lastButtonStates[i] > 0) {
        emit released(i);
      }
    }

    // Update lastButtonStates to reflect the current button states
    lastButtonStates = currentButtons;
  }

  QRosSubscriberInterface* interfacePtr() { return &subscriber_; }
  QRosTypedSubscriber<sensor_msgs::msg::Joy> subscriber_;
};


QROS_NS_FOOT
