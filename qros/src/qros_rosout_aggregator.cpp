#include "qros/qros_rosout_aggregator.h"

QROS_NS_HEAD

QRosRosoutAggregator::QRosRosoutAggregator(QObject* parent)
    : QObject(parent)
{}

void QRosRosoutAggregator::setup(rclcpp::Node::SharedPtr node)
{
    // Default reliable+volatile QoS: matches reliable /rosout publishers but
    // skips the transient-local replay so a fresh UI doesn't get flooded with
    // every log message published since each node started.
    sub_ = node->create_subscription<rcl_interfaces::msg::Log>(
        "/rosout", rclcpp::QoS(rclcpp::KeepLast(1000)),
        [this](const rcl_interfaces::msg::Log::SharedPtr msg) {
            onMsg(msg);
        });
}

void QRosRosoutAggregator::setMaxEntries(int v)
{
    if (v <= 0 || maxEntries_ == v) return;
    maxEntries_ = v;

    bool changed = false;
    while (logs_.size() > maxEntries_) {
        logs_.removeFirst();
        changed = true;
    }

    emit maxEntriesChanged();
    if (changed) emit logsChanged();
}

void QRosRosoutAggregator::clear()
{
    if (logs_.isEmpty()) return;
    logs_.clear();
    emit logsChanged();
}

void QRosRosoutAggregator::onMsg(const rcl_interfaces::msg::Log::SharedPtr msg)
{
    const qint64 stampMs =
        static_cast<qint64>(msg->stamp.sec) * 1000 +
        static_cast<qint64>(msg->stamp.nanosec) / 1'000'000;

    QVariantMap entry;
    entry["stamp"]    = stampMs;
    entry["level"]    = static_cast<int>(msg->level);
    entry["name"]     = QString::fromStdString(msg->name);
    entry["msg"]      = QString::fromStdString(msg->msg);
    entry["file"]     = QString::fromStdString(msg->file);
    entry["function"] = QString::fromStdString(msg->function);
    entry["line"]     = static_cast<int>(msg->line);

    logs_.append(entry);
    while (logs_.size() > maxEntries_) logs_.removeFirst();

    emit logsChanged();
}

QROS_NS_FOOT
