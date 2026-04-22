#pragma once

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

// Placed inside any QML component. Automatically registers with the
// "diagnosticsUpdater" context property on construction and unregisters
// on destruction — no explicit wiring needed.
class QRosDiagnosticTask : public QObject, public QQmlParserStatus {
    Q_OBJECT
    Q_INTERFACES(QQmlParserStatus)

    Q_PROPERTY(QString     name    READ name    WRITE setName    NOTIFY nameChanged)
    Q_PROPERTY(int         level   READ level   WRITE setLevel   NOTIFY levelChanged)
    Q_PROPERTY(QString     message READ message WRITE setMessage NOTIFY messageChanged)
    Q_PROPERTY(QVariantMap values  READ values  WRITE setValues  NOTIFY valuesChanged)

public:
    enum Level { OK = 0, WARN = 1, ERROR = 2, STALE = 3 };
    Q_ENUM(Level)

    explicit QRosDiagnosticTask(QObject* parent = nullptr);
    ~QRosDiagnosticTask() override;

    void classBegin() override {}
    void componentComplete() override;

    QString     name()    const { return name_; }
    int         level()   const { return level_; }
    QString     message() const { return message_; }
    QVariantMap values()  const { return values_; }

    void setName(const QString& v)    { if (name_    != v) { name_    = v; emit nameChanged();    } }
    void setLevel(int v)              { if (level_   != v) { level_   = v; emit levelChanged();   } }
    void setMessage(const QString& v) { if (message_ != v) { message_ = v; emit messageChanged(); } }
    void setValues(const QVariantMap& v) { values_ = v; emit valuesChanged(); }

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


// Context-property singleton. Instantiate in main(), call setup(), and
// register as "diagnosticsUpdater". QRosDiagnosticTask items anywhere in
// the QML tree self-register and self-unregister against it.
class QRosDiagnosticsUpdater : public QObject {
    Q_OBJECT

    Q_PROPERTY(QString hardwareId READ hardwareId WRITE setHardwareId NOTIFY hardwareIdChanged)
    Q_PROPERTY(double  period     READ period     WRITE setPeriod     NOTIFY periodChanged)

public:
    explicit QRosDiagnosticsUpdater(QObject* parent = nullptr);

    void setup(rclcpp::Node::SharedPtr node);

    void registerTask(QRosDiagnosticTask* task);
    void unregisterTask(QRosDiagnosticTask* task);

    QString hardwareId() const { return hardwareId_; }
    double  period()     const { return period_; }

    void setHardwareId(const QString& v) { if (hardwareId_ != v) { hardwareId_ = v; emit hardwareIdChanged(); } }
    void setPeriod(double v);

signals:
    void hardwareIdChanged();
    void periodChanged();

private slots:
    void publish();

private:
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_;
    QTimer  timer_;
    QString hardwareId_ = "roship_ui";
    double  period_     = 1.0;
    QVector<QRosDiagnosticTask*> tasks_;
};

QROS_NS_FOOT
