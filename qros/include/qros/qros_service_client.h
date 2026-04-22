#pragma once

/**
 * @file qros_service_client.h
 * @brief Service client base classes for the qros library.
 *
 * Provides two layers:
 *  - QRosTypedServiceClient<srv_T> — type-erased C++ template that owns the
 *    rclcpp client, request buffer, and response buffer.
 *  - QRosServiceClient             — QObject base exposed to QML; subclasses
 *    return a QRosTypedServiceClient via interfacePtr() and override
 *    onResponseReceived() to emit type-specific signals.
 */

#include "qros_object.h"
#include <QDebug>

QROS_NS_HEAD

/**
 * @brief Abstract interface for all typed service clients.
 *
 * Provides a uniform, type-erased API so QRosServiceClient can drive any
 * service type without knowing it at compile time.
 */
class QRosServiceClientInterface {
public:
  typedef std::shared_ptr<QRosServiceClientInterface> SharedPtr;

  /// Binds the client to the rclcpp node held by @p node.
  virtual void setNode(QRosNode* node) = 0;

  /**
   * @brief Creates the underlying rclcpp client.
   * @param service_name  Fully-qualified ROS service name.
   */
  virtual void createServiceClient(QString service_name) = 0;

  /// Returns the fully-resolved service name.
  virtual QString getServiceName() = 0;

  /**
   * @brief Sends the current request and blocks until a response arrives.
   *
   * Waits up to 1 second for the service to become available, then sends
   * the request synchronously.  Logs an error and returns immediately if the
   * service is unavailable or the call times out.
   */
  virtual void callService() = 0;

  /**
   * @brief Registers the callback invoked on every successful response.
   * @param callback  Zero-argument callable; typically a lambda that
   *                  calls handleResponse() on the owning QRosServiceClient.
   */
  virtual void setCallback(std::function<void()> callback) = 0;
};

/**
 * @brief Concrete, type-specific service client template.
 *
 * Owns the rclcpp::Client<srv_T>, a pre-allocated request buffer, and the
 * last response buffer.  Populate requestBuffer() fields before calling
 * callService(); read results from responseBuffer() inside the callback.
 *
 * ### Blocking behaviour
 * callService() calls `wait_for_service(1s)` then `spin_until_future_complete`.
 * It must **not** be called on the Qt main thread while the ROS timer is
 * spinning — doing so will deadlock.  Call it from QML only if you are
 * certain the executor is not simultaneously waiting.
 *
 * @tparam srv_T  ROS 2 service type (e.g. std_srvs::srv::Trigger).
 */
template <typename srv_T>
class QRosTypedServiceClient : public QRosServiceClientInterface {
public:
  /// @copydoc QRosServiceClientInterface::setNode
  void setNode(QRosNode* node) {
    ros_node_ptr_ = node->getNodePtr();
  }

  /// @copydoc QRosServiceClientInterface::createServiceClient
  void createServiceClient(QString service_name) {
    ros_client_ = ros_node_ptr_->template create_client<srv_T>(service_name.toStdString());
  }

  /// Returns the fully-resolved service name from the underlying rclcpp client.
  QString getServiceName() {
    return QString::fromStdString(ros_client_->get_service_name());
  }

  /// @copydoc QRosServiceClientInterface::callService
  void callService() {
    if (ros_client_->wait_for_service(std::chrono::seconds(1))) {
      auto result_future = ros_client_->async_send_request(request_);
      if (rclcpp::spin_until_future_complete(ros_node_ptr_, result_future) == rclcpp::FutureReturnCode::SUCCESS) {
        response_ = result_future.get();
        callback_();
      } else {
        RCLCPP_ERROR(ros_node_ptr_->get_logger(), "Service call failed");
      }
    } else {
      RCLCPP_ERROR(ros_node_ptr_->get_logger(), "Service not available");
    }
  }

  /**
   * @brief Returns a mutable shared pointer to the outgoing request.
   *
   * Populate fields on this object before calling callService().
   */
  typename srv_T::Request::SharedPtr requestBuffer() {
    return request_;
  }

  /// Returns a const copy of the request buffer.
  const typename srv_T::Request::SharedPtr getConstRequestBuffer() const {
    return request_;
  }

  /**
   * @brief Returns a shared pointer to the last received response.
   *
   * Valid only after at least one successful callService() invocation.
   */
  typename srv_T::Response::SharedPtr responseBuffer() {
    return response_;
  }

  /// Returns a const copy of the response buffer.
  const typename srv_T::Response::SharedPtr getConstResponseBuffer() const {
    return response_;
  }

  /// @copydoc QRosServiceClientInterface::setCallback
  void setCallback(std::function<void()> callback) {
    callback_ = callback;
  }

private:
  typename srv_T::Request::SharedPtr request_ = std::make_shared<typename srv_T::Request>();
  typename srv_T::Response::SharedPtr response_ = std::make_shared<typename srv_T::Response>();
  rclcpp::Node::SharedPtr ros_node_ptr_;
  typename rclcpp::Client<srv_T>::SharedPtr ros_client_;
  std::function<void()> callback_;
};

/**
 * @brief QML-exposed base class for all qros service clients.
 *
 * Concrete subclasses (e.g. QRosTriggerServiceClient) implement interfacePtr()
 * to return their QRosTypedServiceClient<srv_T>, populate request fields as
 * Q_PROPERTYs, and override onResponseReceived() to emit type-specific signals.
 *
 * ### QML usage
 * @code{.qml}
 * QRosTriggerServiceClient {
 *     id:          resetClient
 *     node:        applicationNode
 *     serviceName: "/reset_odometry"
 *     onResponseReceived: console.log("success:", respSuccess, respMessage)
 * }
 * Button {
 *     onClicked: resetClient.callService()
 * }
 * @endcode
 */
class QRosServiceClient : public QRosObject {
  Q_OBJECT
public:
  /// ROS service name.  Setting this creates the rclcpp client.
  Q_PROPERTY(QString serviceName READ getServiceName WRITE setServiceName NOTIFY serviceNameChanged)

  QRosServiceClient(){
    connect(this, &QRosObject::nodeChanged, this, &::QRosServiceClient::setup);
  }
public slots:
  /**
   * @brief Sets the service name and creates the rclcpp client.
   * @param service_name  Fully-qualified ROS service name; empty string is ignored.
   */
  void setServiceName(QString service_name) {
    if (service_name == "") {
      // Service name not yet initialized, just return
      return;
    }
    service_name_ = service_name;
    setup();
  }

  /// Internal slot: (re)creates the client when the node or service name changes.
  void setup(){
    auto node =  getNode();
    if(!node){
      return;
    }
    if (service_name_ == "") {
      // Service name not yet initialized, just return
      return;
    }
    try {
      interfacePtr()->setNode(node);
      interfacePtr()->setCallback([this]() { handleResponse(); });
      interfacePtr()->createServiceClient(service_name_);
      emit serviceNameChanged();
    } catch (...) {
      qWarning() << "Invalid service client name: " << service_name_;
    }
  }

  /// Returns the current service name string.
  QString getServiceName() const {
    return service_name_;
  }

  /// Sends the current request buffer to the service.  Blocks until complete.
  void callService() {
    auto interface = interfacePtr();
    if (interface) {
      interface->callService();
    } else {
      RCLCPP_ERROR(getRosNode()->get_logger(), "Invalid interface pointer passed to QRosServiceClient::callService()");
    }
  }

signals:
  /// Emitted when the serviceName property changes.
  void serviceNameChanged();
  /// Emitted on every successful response.
  void responseReceived();

protected:
  /// Called internally on each response; emits responseReceived() then onResponseReceived().
  virtual void handleResponse() {
    emit responseReceived();
    onResponseReceived();
  }

  /**
   * @brief Override in concrete subclasses to emit type-specific signals.
   *
   * Called after responseReceived() on every successful response.
   * Response data is available via interfacePtr()->responseBuffer().
   */
  virtual void onResponseReceived() = 0;

  /**
   * @brief Returns a pointer to the concrete typed service client interface.
   *
   * Implemented by every concrete subclass to return its
   * QRosTypedServiceClient<srv_T> member.
   */
  virtual QRosServiceClientInterface* interfacePtr() = 0;

private:
  QString service_name_;
};

QROS_NS_FOOT
