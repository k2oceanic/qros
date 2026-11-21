#include "qros/qros_diagnostics_aggregator.h"

QROS_NS_HEAD

QRosDiagnosticsAggregator::QRosDiagnosticsAggregator(QObject* parent)
    : QObject(parent)
{
    staleTimer_.setInterval(1000);
    connect(&staleTimer_, &QTimer::timeout,
            this, &QRosDiagnosticsAggregator::checkStaleness);

    // Coalescing window: batches a burst of inbound messages into one refresh.
    emitTimer_.setSingleShot(true);
    emitTimer_.setInterval(100);
    connect(&emitTimer_, &QTimer::timeout,
            this, &QRosDiagnosticsAggregator::flush);
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
    bool dirty = false;

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

        auto it = entries_.find(key);
        const bool entryChanged = (it == entries_.end())  // new entry
                               || it->stale               // was displayed STALE, now fresh
                               || it->data != map;         // content actually changed
        entries_.insert(key, Entry{ map, std::chrono::steady_clock::now(), false });
        if (entryChanged)
            dirty = true;
    }

    if (dirty)
        markDirty();
}

void QRosDiagnosticsAggregator::checkStaleness()
{
    const auto now = std::chrono::steady_clock::now();
    bool changed = false;

    for (auto& entry : entries_) {
        const double age =
            std::chrono::duration<double>(now - entry.lastSeen).count();
        const bool isStale = age > staleTimeout_;
        if (isStale != entry.stale) {
            entry.stale = isStale;
            changed = true;
        }
    }

    if (changed)
        markDirty();
}

void QRosDiagnosticsAggregator::markDirty()
{
    dirty_ = true;
    if (!emitTimer_.isActive())
        emitTimer_.start();
}

void QRosDiagnosticsAggregator::flush()
{
    if (!dirty_) return;
    dirty_ = false;
    rebuildStatus();
    emit statusChanged();
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

    // Stale entries display as level 3 without mutating the stored live level.
    auto displayedLevel = [](const Entry& e) -> int {
        return e.stale ? 3 : e.data.value("level").toInt();
    };

    QList<QPair<int, QString>> order;  // (priority, key)
    for (auto it = entries_.cbegin(); it != entries_.cend(); ++it) {
        order.append({ priority(displayedLevel(*it)), it.key() });
    }

    std::sort(order.begin(), order.end(),
              [](const auto& a, const auto& b) {
                  return a.first != b.first ? a.first < b.first
                                            : a.second < b.second;
              });

    status_.clear();
    for (const auto& [prio, key] : order) {
        const Entry& e = entries_[key];
        QVariantMap m = e.data;
        if (e.stale)
            m["level"] = 3;
        status_.append(m);
    }
}

QROS_NS_FOOT
