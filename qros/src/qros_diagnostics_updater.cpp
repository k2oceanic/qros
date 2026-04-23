#include "qros/qros_diagnostics_updater.h"
#include <QQmlContext>
#include <QQmlEngine>

QROS_NS_HEAD

// ---------------------------------------------------------------------------
// QRosDiagnosticTask
// ---------------------------------------------------------------------------

QRosDiagnosticTask::QRosDiagnosticTask(QObject* parent)
    : QObject(parent) {}

QRosDiagnosticTask::~QRosDiagnosticTask() {
    if (updater_)
        updater_->unregisterTask(this);
}

void QRosDiagnosticTask::componentComplete() {
    QQmlContext* ctx = qmlContext(this);
    if (!ctx) return;

    auto* obj = ctx->contextProperty("diagnosticsUpdater").value<QObject*>();
    auto* updater = qobject_cast<QRosDiagnosticsUpdater*>(obj);
    if (updater) {
        updater_ = updater;
        updater->registerTask(this);
    }
}

diagnostic_msgs::msg::DiagnosticStatus QRosDiagnosticTask::toStatus() const {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.hardware_id = hardwareId_.toStdString();
    status.name        = name_.toStdString();
    status.level       = static_cast<int8_t>(level_);
    status.message     = message_.toStdString();

    for (auto it = values_.cbegin(); it != values_.cend(); ++it) {
        diagnostic_msgs::msg::KeyValue kv;
        kv.key   = it.key().toStdString();
        kv.value = it.value().toString().toStdString();
        status.values.push_back(kv);
    }

    return status;
}

// ---------------------------------------------------------------------------
// QRosDiagnosticsUpdater
// ---------------------------------------------------------------------------

QRosDiagnosticsUpdater::QRosDiagnosticsUpdater(QObject* parent)
    : QObject(parent) {
    connect(&timer_, &QTimer::timeout, this, &QRosDiagnosticsUpdater::publish);
}

void QRosDiagnosticsUpdater::setup(rclcpp::Node::SharedPtr node) {
    pub_ = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics", rclcpp::QoS(10));
    timer_.start(static_cast<int>(period_ * 1000.0));
}

void QRosDiagnosticsUpdater::setPeriod(double v) {
    if (period_ == v) return;
    period_ = v;
    if (timer_.isActive())
        timer_.setInterval(static_cast<int>(period_ * 1000.0));
    emit periodChanged();
}

void QRosDiagnosticsUpdater::registerTask(QRosDiagnosticTask* task) {
    if (task && !tasks_.contains(task))
        tasks_.append(task);
}

void QRosDiagnosticsUpdater::unregisterTask(QRosDiagnosticTask* task) {
    tasks_.removeAll(task);
}

void QRosDiagnosticsUpdater::publish() {
    if (!pub_) return;

    diagnostic_msgs::msg::DiagnosticArray msg;
    msg.header.stamp = rclcpp::Clock().now();

    for (auto* task : tasks_)
        msg.status.push_back(task->toStatus());

    pub_->publish(msg);
}

QROS_NS_FOOT
