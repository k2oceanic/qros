#pragma once

#include "qros_object.h"
#include <QDebug>

QROS_NS_HEAD

    class QRosServiceClientInterface {
public:
  typedef std::shared_ptr<QRosServiceClientInterface> SharedPtr;
  virtual void setNode(QRosNode* node) = 0;
  virtual void createServiceClient(QString service_name) = 0;
  virtual QString getServiceName() = 0;
  virtual void callService() = 0;
  virtual void setCallback(std::function<void()> callback) = 0;
};

template <typename srv_T>
class QRosTypedServiceClient : public QRosServiceClientInterface {
public:
  void setNode(QRosNode* node) {
    ros_node_ptr_ = node->getNodePtr();
  }

  void createServiceClient(QString service_name) {
    ros_client_ = ros_node_ptr_->template create_client<srv_T>(service_name.toStdString());
  }

  QString getServiceName() {
    return QString::fromStdString(ros_client_->get_service_name());
  }

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

  typename srv_T::Request::SharedPtr requestBuffer() {
    return request_;
  }

  const typename srv_T::Request::SharedPtr getConstRequestBuffer() const {
    return request_;
  }

  typename srv_T::Response::SharedPtr responseBuffer() {
    return response_;
  }

  const typename srv_T::Response::SharedPtr getConstResponseBuffer() const {
    return response_;
  }

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

class QRosServiceClient : public QRosObject {
  Q_OBJECT
public:
  Q_PROPERTY(QString serviceName READ getServiceName WRITE setServiceName NOTIFY serviceNameChanged)

  QRosServiceClient(){
    connect(this, &QRosObject::nodeChanged, this, &::QRosServiceClient::setup);
  }
public slots:
  void setServiceName(QString service_name) {
    if (service_name == "") {
      // Service name not yet initialized, just return
      return;
    }
    service_name_ = service_name;
    setup();
  }

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

  QString getServiceName() const {
    return service_name_;
  }

  void callService() {
    auto interface = interfacePtr();
    if (interface) {
      interface->callService();
    } else {
      RCLCPP_ERROR(getRosNode()->get_logger(), "Invalid interface pointer passed to QRosServiceClient::callService()");
    }
  }

signals:
  void serviceNameChanged();
  void responseReceived();

protected:
  virtual void handleResponse() {
    emit responseReceived();
    onResponseReceived();
  }

  virtual void onResponseReceived() = 0;
  virtual QRosServiceClientInterface* interfacePtr() = 0;

private:
  QString service_name_;
};

QROS_NS_FOOT
