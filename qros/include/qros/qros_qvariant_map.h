#pragma once

#include "qros_publisher.h"
#include "qros_subscriber.h"
#include "qros_qvariant.h"  // rosToQVariant / qvariantToRos helpers
#include <qros_interfaces/msg/q_variant_map.hpp>
#include <QString>
#include <QStringList>
#include <QVariant>
#include <QVariantMap>

QROS_NS_HEAD

// ---------------------------------------------------------------------------
// Publisher
// ---------------------------------------------------------------------------

class QRosQVariantMapPublisher : public QRosPublisher {
    Q_OBJECT
public:
    Q_PROPERTY(QStringList keys READ getKeys NOTIFY dataChanged)

public slots:
    QStringList getKeys() {
        QStringList list;
        for (const auto& k : publisher_.msgBuffer().keys)
            list << QString::fromStdString(k);
        return list;
    }

    /// Replace the entire map contents from a QVariantMap.
    void setMap(const QVariantMap& map) {
        auto& msg = publisher_.msgBuffer();
        msg.keys.clear();
        msg.values.clear();
        for (auto it = map.cbegin(); it != map.cend(); ++it) {
            msg.keys.push_back(it.key().toStdString());
            qros_interfaces::msg::QVariant v;
            qvariantToRos(it.value(), v);
            msg.values.push_back(v);
        }
        emit dataChanged();
    }

    /// Insert or update a single key/value pair.
    void setValue(const QString& key, const QVariant& value) {
        auto& msg = publisher_.msgBuffer();
        const std::string k = key.toStdString();
        for (size_t i = 0; i < msg.keys.size(); ++i) {
            if (msg.keys[i] == k) {
                qvariantToRos(value, msg.values[i]);
                emit dataChanged();
                return;
            }
        }
        msg.keys.push_back(k);
        qros_interfaces::msg::QVariant v;
        qvariantToRos(value, v);
        msg.values.push_back(v);
        emit dataChanged();
    }

    /// Remove a key/value pair by key. No-op if the key does not exist.
    void removeKey(const QString& key) {
        auto& msg = publisher_.msgBuffer();
        const std::string k = key.toStdString();
        for (size_t i = 0; i < msg.keys.size(); ++i) {
            if (msg.keys[i] == k) {
                msg.keys.erase(msg.keys.begin() + i);
                msg.values.erase(msg.values.begin() + i);
                emit dataChanged();
                return;
            }
        }
    }

    void clear() {
        publisher_.msgBuffer().keys.clear();
        publisher_.msgBuffer().values.clear();
        emit dataChanged();
    }

signals:
    void dataChanged();

protected:
    QRosPublisherInterface* interfacePtr() override { return &publisher_; }
    QRosTypedPublisher<qros_interfaces::msg::QVariantMap> publisher_;
};

// ---------------------------------------------------------------------------
// Subscriber
// ---------------------------------------------------------------------------

class QRosQVariantMapSubscriber : public QRosSubscriber {
    Q_OBJECT
public:
    Q_PROPERTY(QStringList keys READ getKeys NOTIFY dataChanged)
    /// The full map as a QVariantMap — the most convenient property for QML (map.someKey).
    Q_PROPERTY(QVariantMap map READ getMap NOTIFY dataChanged)

public slots:
    QStringList getKeys() {
        QStringList list;
        for (const auto& k : subscriber_.msgBuffer().keys)
            list << QString::fromStdString(k);
        return list;
    }

    QVariantMap getMap() {
        QVariantMap result;
        const auto& msg = subscriber_.msgBuffer();
        for (size_t i = 0; i < msg.keys.size() && i < msg.values.size(); ++i)
            result[QString::fromStdString(msg.keys[i])] = rosToQVariant(msg.values[i]);
        return result;
    }

    /// Look up a single value by key. Returns an invalid QVariant if not found.
    QVariant value(const QString& key) {
        const auto& msg = subscriber_.msgBuffer();
        const std::string k = key.toStdString();
        for (size_t i = 0; i < msg.keys.size() && i < msg.values.size(); ++i) {
            if (msg.keys[i] == k)
                return rosToQVariant(msg.values[i]);
        }
        return QVariant();
    }

    bool containsKey(const QString& key) {
        const std::string k = key.toStdString();
        for (const auto& existing : subscriber_.msgBuffer().keys) {
            if (existing == k) return true;
        }
        return false;
    }

signals:
    void dataChanged();

protected:
    void onMsgReceived() override { emit dataChanged(); }

private:
    QRosSubscriberInterface* interfacePtr() override { return &subscriber_; }
    QRosTypedSubscriber<qros_interfaces::msg::QVariantMap> subscriber_;
};

QROS_NS_FOOT
