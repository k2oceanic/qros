#pragma once

#include <QByteArray>
#include <QDataStream>
#include <QIODevice>
#include <QVariant>
#include "qros_publisher.h"
#include "qros_subscriber.h"
#include <qdebug.h>
#include <qros_interfaces/msg/q_variant.hpp>

QROS_NS_HEAD

// ---------------------------------------------------------------------------
// Conversion helpers (reused by qros_qvariant_map.h)
// ---------------------------------------------------------------------------

inline QVariant rosToQVariant(const qros_interfaces::msg::QVariant& msg)
{
    QByteArray ba(reinterpret_cast<const char*>(msg.data.data()), static_cast<int>(msg.data.size()));
    QDataStream stream(&ba, QIODevice::ReadOnly);
    QVariant v;
    stream >> v;
    return v;
}

inline void qvariantToRos(const QVariant& v, qros_interfaces::msg::QVariant& msg)
{
    QByteArray ba;
    QDataStream stream(&ba, QIODevice::WriteOnly);
    stream << v;
    msg.data.assign(reinterpret_cast<const uint8_t*>(ba.constData()),
                    reinterpret_cast<const uint8_t*>(ba.constData()) + ba.size());
}

// ---------------------------------------------------------------------------
// Publisher
// ---------------------------------------------------------------------------

class QRosQVariantPublisher : public QRosPublisher {
    Q_OBJECT
public:
    Q_PROPERTY(QVariant value READ getValue WRITE setValue NOTIFY dataChanged)

public slots:
    QVariant getValue() { return rosToQVariant(publisher_.msgBuffer()); }
    void setValue(const QVariant& v) {
        qvariantToRos(v, publisher_.msgBuffer());
        emit dataChanged();
    }

signals:
    void dataChanged();

protected:
    QRosPublisherInterface* interfacePtr() override { return &publisher_; }
    QRosTypedPublisher<qros_interfaces::msg::QVariant> publisher_;
};

// ---------------------------------------------------------------------------
// Subscriber
// ---------------------------------------------------------------------------

class QRosQVariantSubscriber : public QRosSubscriber {
    Q_OBJECT
public:
    Q_PROPERTY(QVariant value READ getValue NOTIFY dataChanged)

public slots:
    QVariant getValue() { return rosToQVariant(subscriber_.msgBuffer()); }

signals:
    void dataChanged();

protected:
    void onMsgReceived() override { emit dataChanged(); }

private:
    QRosSubscriberInterface* interfacePtr() override { return &subscriber_; }
    QRosTypedSubscriber<qros_interfaces::msg::QVariant> subscriber_;
};

QROS_NS_FOOT
