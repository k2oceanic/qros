#pragma once

#include <QObject>
#include <QSettings>

QROS_NS_HEAD

class QRosSettings : public QObject
{
    Q_OBJECT
public:
    explicit QRosSettings(const QString &organization, const QString &application, QObject *parent = nullptr)
        : QObject(parent), m_settings(organization, application)
    {
    }

    Q_INVOKABLE void setValue(const QString &key, const QVariant &value)
    {
        m_settings.setValue(key, value);
    }

    Q_INVOKABLE QVariant getValue(const QString &key, const QVariant &defaultValue = QVariant()) const
    {
        return m_settings.value(key, defaultValue);
    }

private:
    QSettings m_settings;
};

QROS_NS_FOOT

