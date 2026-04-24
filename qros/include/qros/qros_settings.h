#pragma once

/**
 * @file qros_settings.h
 * @brief Thin QML-invokable wrapper around QSettings for persistent key-value storage.
 */

#include <QObject>
#include <QSettings>

QROS_NS_HEAD

/**
 * @brief QML-accessible persistent settings store backed by QSettings.
 *
 * Stores configuration in `~/.config/<organization>/<application>.conf`.
 * Instantiate from C++ and expose as a context property for use in QML.
 *
 * @note For purely QML-side persistence, Qt.labs.settings is often more
 * convenient.  Use QRosSettings when settings also need to be read or
 * written from C++ code.
 *
 * ### C++ setup
 * @code{.cpp}
 * QRosSettings* settings = new QRosSettings("RoShip", "engineering", &app);
 * engine.rootContext()->setContextProperty("appSettings", settings);
 * @endcode
 *
 * ### QML usage
 * @code{.qml}
 * Component.onCompleted: mySlider.value = appSettings.getValue("gain", 1.0)
 * onValueChanged:        appSettings.setValue("gain", mySlider.value)
 * @endcode
 */
class QRosSettings : public QObject
{
    Q_OBJECT
public:
    /**
     * @brief Constructs the settings store.
     * @param organization  Organization name (used in the config file path).
     * @param application   Application name (used in the config file path).
     * @param parent        Optional Qt parent.
     */
    explicit QRosSettings(const QString &organization, const QString &application, QObject *parent = nullptr)
        : QObject(parent), m_settings(organization, application)
    {
    }

    /**
     * @brief Writes a key-value pair to persistent storage.
     * @param key    Settings key.
     * @param value  Value to store (any QVariant-compatible type).
     */
    Q_INVOKABLE void setValue(const QString &key, const QVariant &value)
    {
        m_settings.setValue(key, value);
    }

    /**
     * @brief Reads a value from persistent storage.
     * @param key           Settings key.
     * @param defaultValue  Value to return if the key does not exist.
     * @return The stored value, or @p defaultValue if not found.
     */
    Q_INVOKABLE QVariant getValue(const QString &key, const QVariant &defaultValue = QVariant()) const
    {
        return m_settings.value(key, defaultValue);
    }

private:
    QSettings m_settings;
};

QROS_NS_FOOT
