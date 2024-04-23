#include "qros.h"

#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QQuickStyle>

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto ros_node = std::make_shared<rclcpp::Node>("qml_example");

  qputenv("QT_QUICK_CONTROLS_CONF", "://QtQuickControls2.conf");
  #if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
  #endif
    QGuiApplication app(argc, argv);

  qros::registerQmlTypes();

  QRosNode applicationNode;
  QQmlApplicationEngine engine;

  const QUrl url(QStringLiteral("qrc:/qml/ExampleWindow.qml"));
  QObject::connect(&engine, &QQmlApplicationEngine::objectCreated,
      &app, [url](QObject *obj, const QUrl &objUrl) {
        if (!obj && url == objUrl)
          QCoreApplication::exit(-1);
      }, Qt::QueuedConnection);

  QQmlContext * rootContext = engine.rootContext();

  applicationNode.setNodePtr(ros_node);

  rootContext->setContextProperty("applicationNode", &applicationNode);

  engine.load(url);

  applicationNode.spinRosWithTimer();
  auto out = app.exec();
  rclcpp::shutdown();
  return out;

}
