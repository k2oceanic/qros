#pragma once

/**
 * @file qros_qvariant.h
 * @brief Publisher and subscriber for qros_interfaces/msg/QVariant.
 *
 * Enables sending arbitrary QML values over ROS topics by serialising
 * them through QDataStream into the message's byte array payload.
 * Supports all QVariant-compatible types (int, double, bool, QString,
 * QVector, QVariantMap, etc.).
 *
 * The complementary QRosQVariantMapPublisher / Subscriber in qros_qvariant_map.h
 * add a string-keyed dictionary wrapper on top of this mechanism.
 */

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

/**
 * @brief Deserialises a QVariant from a QVariant ROS message.
 * @param msg  Source message containing the QDataStream-encoded byte array.
 * @return The deserialised QVariant value.
 */
inline QVariant rosToQVariant(const qros_interfaces::msg::QVariant& msg)
{
    QByteArray ba(reinterpret_cast<const char*>(msg.data.data()), static_cast<int>(msg.data.size()));
    QDataStream stream(&ba, QIODevice::ReadOnly);
    QVariant v;
    stream >> v;
    return v;
}

/**
 * @brief Serialises a QVariant into a QVariant ROS message.
 * @param v    Value to serialise.
 * @param msg  Destination message; its data field is replaced.
 */
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

/**
 * @brief Publishes `qros_interfaces/QVariant` messages.
 *
 * Serialises any QVariant-compatible QML value into the ROS message.
 *
 * ### QML usage
 * @code{.qml}
 * QRosQVariantPublisher {
 *     node:  applicationNode
 *     topic: "/shared_state"
 *     value: someComplexObject
 * }
 * @endcode
 */
class QRosQVariantPublisher : public QRosPublisher {
    Q_OBJECT
public:
    /// The value to serialise and publish.
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

/**
 * @brief Subscribes to `qros_interfaces/QVariant` messages.
 *
 * Deserialises each received message back into a QVariant for QML use.
 *
 * ### QML usage
 * @code{.qml}
 * QRosQVariantSubscriber {
 *     node:  applicationNode
 *     topic: "/shared_state"
 *     onDataChanged: handleUpdate(value)
 * }
 * @endcode
 */
class QRosQVariantSubscriber : public QRosSubscriber {
    Q_OBJECT
public:
    /// The last received deserialised value.
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
