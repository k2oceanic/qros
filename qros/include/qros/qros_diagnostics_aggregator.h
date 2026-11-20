#pragma once

/**
 * @file qros_diagnostics_aggregator.h
 * @brief In-process aggregator for /diagnostics.
 *
 * Subscribes to `/diagnostics`, merges entries by (hardware_id, name), tracks
 * staleness, and exposes the result as a QVariantList for QML.  Register once
 * as the `"diagnosticsAggregator"` context property.
 *
 * Each entry in `status` is a QVariantMap with the same keys as
 * QRosDiagnosticArraySubscriber: level, name, message, hardware_id, values.
 * Entries not updated within `staleTimeoutSeconds` are promoted to level 3 (STALE).
 *
 * Sorted order: ERROR → WARN → STALE → OK, then alphabetically by name.
 */

#include "qros_defs.h"
#include <QObject>
#include <QTimer>
#include <QVariantList>
#include <QMap>
#include <QString>
#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <chrono>

QROS_NS_HEAD

class QRosDiagnosticsAggregator : public QObject {
    Q_OBJECT

    Q_PROPERTY(QVariantList status
               READ status NOTIFY statusChanged)

    Q_PROPERTY(double staleTimeoutSeconds
               READ staleTimeoutSeconds
               WRITE setStaleTimeoutSeconds
               NOTIFY staleTimeoutSecondsChanged)

public:
    explicit QRosDiagnosticsAggregator(QObject* parent = nullptr);

    void setup(rclcpp::Node::SharedPtr node);

    QVariantList status()             const { return status_; }
    double       staleTimeoutSeconds() const { return staleTimeout_; }

    void setStaleTimeoutSeconds(double v);

signals:
    void statusChanged();
    void staleTimeoutSecondsChanged();

private slots:
    void checkStaleness();

private:
    struct Entry {
        QVariantMap                              data;
        std::chrono::steady_clock::time_point    lastSeen;
        int                                      liveLevel = 0;
    };

    void onMsg(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg);
    void rebuildStatus();

    rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr sub_;
    QMap<QString, Entry> entries_;
    QVariantList         status_;
    QTimer               staleTimer_;
    double               staleTimeout_ = 3.0;
};

QROS_NS_FOOT
