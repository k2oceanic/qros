#pragma once

#include <QObject>
#include <QSettings>

QROS_NS_HEAD

/**
 * @brief This class is a QML wrapper for QSettings. 
 */
class QRosSettings : public QObject
{
    Q_OBJECT
public:
    /**
     * @brief Constructor for QRosSettings object
     * 
     * @param organization The organization name
     * @param application The application name
     * @note The config file will be stored in .config/organization/application.conf
     */
    explicit QRosSettings(const QString &organization, const QString &application, QObject *parent = nullptr)
        : QObject(parent), m_settings(organization, application)
    {
    }
    /**
     * @brief Set the value of a key in the settings
     * 
     * @param key The key to set the value of
     * @param value The value to set
     */
    Q_INVOKABLE void setValue(const QString &key, const QVariant &value)
    {
        m_settings.setValue(key, value);
    }

    /**
     * @brief Get the value of a key from the settings
     * 
     * @param key The key to get the value of
     * @param defaultValue The default value to return if the key does not exist
     * @return QVariant The value of the key
     */
    Q_INVOKABLE QVariant getValue(const QString &key, const QVariant &defaultValue = QVariant()) const
    {
        return m_settings.value(key, defaultValue);
    }

private:
    QSettings m_settings;
};

QROS_NS_FOOT

