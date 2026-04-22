#pragma once

/**
 * @file qros_diagnostics_updater.h
 * @brief QML-driven aggregator for ROS 2 /diagnostics output.
 *
 * Provides two cooperating classes:
 *  - QRosDiagnosticTask        — inline QML element that self-registers with the updater
 *  - QRosDiagnosticsUpdater    — context-property singleton that periodically
 *                                 publishes all registered tasks as a DiagnosticArray
 */

#include <QObject>
#include <QTimer>
#include <QVector>
#include <QPointer>
#include <QVariantMap>
#include <QQmlParserStatus>
#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include "qros_node.h"

QROS_NS_HEAD

class QRosDiagnosticsUpdater;

/**
 * @brief Self-registering diagnostic task for use inside any QML component.
 *
 * Place a QRosDiagnosticTask inside any QML component that has health state
 * to report.  On componentComplete() it automatically finds the
 * `"diagnosticsUpdater"` context property and registers itself; on
 * destruction it unregisters safely via a QPointer.
 *
 * Bind `name`, `level`, `message`, and `values` to the component's own
 * properties — the updater reads them when publishing the DiagnosticArray.
 *
 * ### QML usage
 * @code{.qml}
 * // Inside BatteryDisplay.qml
 * QRosDiagnosticTask {
 *     name:    "Battery Voltage"
 *     level:   volts < 20 ? QRosDiagnosticTask.ERROR : QRosDiagnosticTask.OK
 *     message: "%.1f V".arg(volts)
 *     values:  ({ "voltage_v": volts, "raw_adc": rawCounts })
 * }
 * @endcode
 */
class QRosDiagnosticTask : public QObject, public QQmlParserStatus {
    Q_OBJECT
    Q_INTERFACES(QQmlParserStatus)

    /// Human-readable task name (appears as the status name in the DiagnosticArray).
    Q_PROPERTY(QString     name    READ name    WRITE setName    NOTIFY nameChanged)
    /// Severity level; use the Level enum constants (OK, WARN, ERROR, STALE).
    Q_PROPERTY(int         level   READ level   WRITE setLevel   NOTIFY levelChanged)
    /// Short description of the current state (e.g. "12.4 V", "sensor timeout").
    Q_PROPERTY(QString     message READ message WRITE setMessage NOTIFY messageChanged)
    /// Optional key/value pairs published as diagnostic KeyValue entries.
    Q_PROPERTY(QVariantMap values  READ values  WRITE setValues  NOTIFY valuesChanged)

public:
    /// Diagnostic severity levels, matching diagnostic_msgs/DiagnosticStatus constants.
    enum Level { OK = 0, WARN = 1, ERROR = 2, STALE = 3 };
    Q_ENUM(Level)

    explicit QRosDiagnosticTask(QObject* parent = nullptr);
    ~QRosDiagnosticTask() override;

    void classBegin() override {}

    /// Called by the QML engine after all properties are bound; self-registers with diagnosticsUpdater.
    void componentComplete() override;

    QString     name()    const { return name_; }
    int         level()   const { return level_; }
    QString     message() const { return message_; }
    QVariantMap values()  const { return values_; }

    void setName(const QString& v)    { if (name_    != v) { name_    = v; emit nameChanged();    } }
    void setLevel(int v)              { if (level_   != v) { level_   = v; emit levelChanged();   } }
    void setMessage(const QString& v) { if (message_ != v) { message_ = v; emit messageChanged(); } }
    void setValues(const QVariantMap& v) { values_ = v; emit valuesChanged(); }

    /// Converts this task's current state to a ROS DiagnosticStatus message.
    diagnostic_msgs::msg::DiagnosticStatus toStatus() const;

signals:
    void nameChanged();
    void levelChanged();
    void messageChanged();
    void valuesChanged();

private:
    QString     name_    = "Unnamed Task";
    int         level_   = OK;
    QString     message_ = "";
    QVariantMap values_;
    QPointer<QRosDiagnosticsUpdater> updater_;
};


/**
 * @brief Context-property singleton that aggregates QML diagnostic tasks and publishes them to /diagnostics.
 *
 * Instantiate once in main(), call setup(), and register as the
 * `"diagnosticsUpdater"` engine context property.  QRosDiagnosticTask
 * items anywhere in the QML tree will find it automatically.
 *
 * ### C++ setup
 * @code{.cpp}
 * QRosDiagnosticsUpdater* diagUpdater = new QRosDiagnosticsUpdater(&engine);
 * diagUpdater->setup(ros_node);
 * engine.rootContext()->setContextProperty("diagnosticsUpdater", diagUpdater);
 * @endcode
 */
class QRosDiagnosticsUpdater : public QObject {
    Q_OBJECT

    /// Hardware ID stamped into every DiagnosticStatus (defaults to the node's fully-qualified name).
    Q_PROPERTY(QString hardwareId READ hardwareId WRITE setHardwareId NOTIFY hardwareIdChanged)
    /// Publish period in seconds (default 1.0 Hz).
    Q_PROPERTY(double  period     READ period     WRITE setPeriod     NOTIFY periodChanged)

public:
    explicit QRosDiagnosticsUpdater(QObject* parent = nullptr);

    /**
     * @brief Initialises the ROS publisher and sets the default hardware ID.
     * @param node  The application's rclcpp node.  The hardware ID defaults to
     *              `node->get_fully_qualified_name()` if not overridden.
     */
    void setup(rclcpp::Node::SharedPtr node);

    /// Adds @p task to the set of tasks included in the next publish cycle.
    void registerTask(QRosDiagnosticTask* task);

    /// Removes @p task from the publish set (called automatically from ~QRosDiagnosticTask).
    void unregisterTask(QRosDiagnosticTask* task);

    QString hardwareId() const { return hardwareId_; }
    double  period()     const { return period_; }

    void setHardwareId(const QString& v) { if (hardwareId_ != v) { hardwareId_ = v; emit hardwareIdChanged(); } }

    /**
     * @brief Sets the publish period and restarts the internal timer.
     * @param v  Period in seconds.
     */
    void setPeriod(double v);

signals:
    void hardwareIdChanged();
    void periodChanged();

private slots:
    /// Timer callback: collects all registered tasks and publishes a DiagnosticArray.
    void publish();

private:
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_;
    QTimer  timer_;
    QString hardwareId_ = "roship_ui";
    double  period_     = 1.0;
    QVector<QRosDiagnosticTask*> tasks_;
};

QROS_NS_FOOT
