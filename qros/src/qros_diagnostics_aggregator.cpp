#include "qros/qros_diagnostics_aggregator.h"

QROS_NS_HEAD

QRosDiagnosticsAggregator::QRosDiagnosticsAggregator(QObject* parent)
    : QObject(parent)
{
    staleTimer_.setInterval(1000);
    connect(&staleTimer_, &QTimer::timeout,
            this, &QRosDiagnosticsAggregator::checkStaleness);
}

void QRosDiagnosticsAggregator::setup(rclcpp::Node::SharedPtr node)
{
    sub_ = node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics", rclcpp::QoS(10),
        [this](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
            onMsg(msg);
        });
    staleTimer_.start();
}

void QRosDiagnosticsAggregator::setStaleTimeoutSeconds(double v)
{
    if (staleTimeout_ == v) return;
    staleTimeout_ = v;
    emit staleTimeoutSecondsChanged();
}

void QRosDiagnosticsAggregator::onMsg(
    const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
{
    for (const auto& s : msg->status) {
        const QString key = QString::fromStdString(s.hardware_id)
                          + ":"
                          + QString::fromStdString(s.name);

        QVariantMap map;
        map["level"]       = static_cast<int>(s.level);
        map["name"]        = QString::fromStdString(s.name);
        map["message"]     = QString::fromStdString(s.message);
        map["hardware_id"] = QString::fromStdString(s.hardware_id);

        QVariantList values;
        for (const auto& kv : s.values) {
            QVariantMap kvMap;
            kvMap["key"]   = QString::fromStdString(kv.key);
            kvMap["value"] = QString::fromStdString(kv.value);
            values.append(kvMap);
        }
        map["values"] = values;

        entries_[key] = Entry{ map, std::chrono::steady_clock::now(),
                               static_cast<int>(s.level) };
    }

    rebuildStatus();
    emit statusChanged();
}

void QRosDiagnosticsAggregator::checkStaleness()
{
    const auto now = std::chrono::steady_clock::now();
    bool changed = false;

    for (auto& entry : entries_) {
        const double age =
            std::chrono::duration<double>(now - entry.lastSeen).count();
        const int displayed = entry.data["level"].toInt();
        const bool isStale  = age > staleTimeout_;

        if (isStale && displayed != 3) {
            entry.data["level"] = 3;
            changed = true;
        } else if (!isStale && displayed == 3 && entry.liveLevel != 3) {
            // Restore real level once fresh data arrives (shouldn't normally
            // happen via the timer, but guards against clock weirdness)
            entry.data["level"] = entry.liveLevel;
            changed = true;
        }
    }

    if (changed) {
        rebuildStatus();
        emit statusChanged();
    }
}

void QRosDiagnosticsAggregator::rebuildStatus()
{
    // Sort priority: ERROR(2) → WARN(1) → STALE(3) → OK(0), then name A-Z
    auto priority = [](int level) -> int {
        switch (level) {
        case 2:  return 0;  // ERROR
        case 1:  return 1;  // WARN
        case 3:  return 2;  // STALE
        default: return 3;  // OK
        }
    };

    QList<QPair<int, QString>> order;  // (priority, key)
    for (auto it = entries_.cbegin(); it != entries_.cend(); ++it) {
        order.append({ priority(it->data["level"].toInt()), it.key() });
    }

    std::sort(order.begin(), order.end(),
              [](const auto& a, const auto& b) {
                  return a.first != b.first ? a.first < b.first
                                            : a.second < b.second;
              });

    status_.clear();
    for (const auto& [prio, key] : order)
        status_.append(entries_[key].data);
}

QROS_NS_FOOT
