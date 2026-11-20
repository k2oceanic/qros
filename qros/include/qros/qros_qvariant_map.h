#pragma once

/**
 * @file qros_qvariant_map.h
 * @brief Publisher and subscriber for qros_interfaces/msg/QVariantMap.
 *
 * A string-keyed dictionary of QVariant values transmitted over a ROS topic.
 * Each entry is serialised via QDataStream (same encoding as QRosQVariantPublisher).
 * This allows QML objects (JSON-like structures) to be shared across nodes.
 */

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

/**
 * @brief Publishes `qros_interfaces/QVariantMap` messages.
 *
 * Wraps a QVariantMap (string → QVariant) for transmission over ROS.
 * Individual entries can be updated with setValue(); the entire map can be
 * replaced with setMap().
 *
 * ### QML usage
 * @code{.qml}
 * QRosQVariantMapPublisher {
 *     node:    applicationNode
 *     topic:   "/ui_state"
 *     latched: true
 *     map: ({
 *         "mode":   currentMode,
 *         "active": isActive,
 *         "gain":   gainSlider.value
 *     })
 *     onTopicChanged: publish()
 *     onDataChanged:  publish()
 * }
 * @endcode
 */
class QRosQVariantMapPublisher : public QRosPublisher {
    Q_OBJECT
public:
  /// Ordered list of keys in the current map.
  Q_PROPERTY(QStringList keys READ getKeys NOTIFY dataChanged)
  /// The full map as a QVariantMap.
  Q_PROPERTY(QVariantMap map READ getMap WRITE setMap NOTIFY dataChanged)

public slots:
    QVariantMap getMap() {
        QVariantMap result;
        const auto& msg = publisher_.msgBuffer();
        for (size_t i = 0; i < msg.keys.size() && i < msg.values.size(); ++i)
            result[QString::fromStdString(msg.keys[i])] = rosToQVariant(msg.values[i]);
        return result;
    }

    QStringList getKeys() {
        QStringList list;
        for (const auto& k : publisher_.msgBuffer().keys)
            list << QString::fromStdString(k);
        return list;
    }

    /// Replaces the entire map contents from a QVariantMap.
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

    /**
     * @brief Inserts or updates a single key/value pair.
     * @param key    Map key string.
     * @param value  Value to associate with the key.
     */
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

    /**
     * @brief Removes a key/value pair by key.  No-op if the key does not exist.
     * @param key  Map key to remove.
     */
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

    /// Clears all entries from the map.
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

/**
 * @brief Subscribes to `qros_interfaces/QVariantMap` messages.
 *
 * ### QML usage
 * @code{.qml}
 * QRosQVariantMapSubscriber {
 *     node:    applicationNode
 *     latched: true
 *     topic:   "/ui_state"
 *     onMsgReceived: {
 *         const m = map
 *         root.mode   = m["mode"]   ?? root.mode
 *         root.active = m["active"] ?? root.active
 *         root.gain   = m["gain"]   ?? root.gain
 *     }
 * }
 * @endcode
 */
class QRosQVariantMapSubscriber : public QRosSubscriber {
    Q_OBJECT
public:
    /// Ordered list of keys from the last received message.
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

    /**
     * @brief Returns a single value by key.
     * @param key  Map key to look up.
     * @return The stored value, or an invalid QVariant if the key is not found.
     */
    QVariant value(const QString& key) {
        const auto& msg = subscriber_.msgBuffer();
        const std::string k = key.toStdString();
        for (size_t i = 0; i < msg.keys.size() && i < msg.values.size(); ++i) {
            if (msg.keys[i] == k)
                return rosToQVariant(msg.values[i]);
        }
        return QVariant();
    }

    /**
     * @brief Returns true if @p key is present in the last received message.
     * @param key  Map key to check.
     */
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
