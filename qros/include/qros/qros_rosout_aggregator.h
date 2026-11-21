#pragma once

/**
 * @file qros_rosout_aggregator.h
 * @brief In-process aggregator for /rosout.
 *
 * Subscribes to `/rosout`, keeps a bounded ring buffer of recent log messages,
 * and exposes the result as a QVariantList for QML.  Register once as the
 * `"rosoutAggregator"` context property.
 *
 * Each entry in `logs` is a QVariantMap with keys:
 *   stamp    (qint64, msec since unix epoch)
 *   level    (int, rcl_interfaces/Log severity: 10/20/30/40/50)
 *   name     (QString, logger name)
 *   msg      (QString, log message)
 *   file     (QString)
 *   function (QString)
 *   line     (int)
 *
 * Subscriber uses a default volatile QoS so newly-attached UIs do not replay
 * the publisher's full transient-local history on launch.
 */

#include "qros_defs.h"
#include <QObject>
#include <QVariantList>
#include <QVariantMap>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/log.hpp>

QROS_NS_HEAD

class QRosRosoutAggregator : public QObject {
    Q_OBJECT

    Q_PROPERTY(QVariantList logs
               READ logs NOTIFY logsChanged)

    Q_PROPERTY(int maxEntries
               READ maxEntries
               WRITE setMaxEntries
               NOTIFY maxEntriesChanged)

public:
    explicit QRosRosoutAggregator(QObject* parent = nullptr);

    void setup(rclcpp::Node::SharedPtr node);

    QVariantList logs()       const { return logs_; }
    int          maxEntries() const { return maxEntries_; }

    void setMaxEntries(int v);

    Q_INVOKABLE void clear();

signals:
    void logsChanged();
    void maxEntriesChanged();

private:
    void onMsg(const rcl_interfaces::msg::Log::SharedPtr msg);

    rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr sub_;
    QVariantList logs_;
    int          maxEntries_ = 1000;
};

QROS_NS_FOOT
