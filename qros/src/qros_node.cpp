#include "qros_node.h"

QROS_NS_HEAD

QRosNode::QRosNode(QObject *parent)
    : QObject{parent}
{}


rclcpp::Node::SharedPtr QRosNode::getNodePtr() const
{
    return node_ptr_;
}

void QRosNode::setNodePtr(const rclcpp::Node::SharedPtr &newNode_ptr)
{
  node_ptr_ = newNode_ptr;
}

rclcpp::ParameterValue QRosNode::paramValueFromQVariant(const QVariant &value)
{
  rclcpp::ParameterValue param_value;
  auto type = value.type();
  if (type == QVariant::List || type ==QVariant::UserType) {
    // Handle lists which can represent vectors
    QVariantList list = value.toList();
    if (!list.isEmpty()) {
      switch (list.first().type()) {
      case QVariant::Int: {
        std::vector<int> int_array;
        for (const QVariant &item : list) int_array.push_back(item.toInt());
        param_value = rclcpp::ParameterValue(int_array);
        break;}
      case QVariant::Double:{
        std::vector<double> double_array;
        for (const QVariant &item : list) double_array.push_back(item.toDouble());
        param_value = rclcpp::ParameterValue(double_array);
        break;}
      case QVariant::String:{
        std::vector<std::string> string_array;
        for (const QVariant &item : list) string_array.push_back(item.toString().toStdString());
        param_value = rclcpp::ParameterValue(string_array);
        break;}
      case QVariant::Bool:{
        std::vector<bool> bool_array;
        for (const QVariant &item : list) bool_array.push_back(item.toBool());
        param_value = rclcpp::ParameterValue(bool_array);
        break;}
      }
    }
  } else {
    // Handle scalar types
    switch (value.type()) {
    case QVariant::Int:
      param_value = rclcpp::ParameterValue(value.toInt());
      break;
    case QVariant::Double:
      param_value = rclcpp::ParameterValue(value.toDouble());
      break;
    case QVariant::String:
      param_value = rclcpp::ParameterValue(value.toString().toStdString());
      break;
    case QVariant::Bool:
      param_value = rclcpp::ParameterValue(value.toBool());
      break;
    default:
      RCLCPP_ERROR(node_ptr_->get_logger(), "Unhandled QVariant type: %s ",value.typeName());
      break;
    }
  }
  return param_value;
}

void QRosNode::spinRosWithTimer(int durration)
{
  ros_timer_ = new QTimer(this);
  connect(ros_timer_, SIGNAL(timeout()), this, SLOT(spinRosSome()));
  ros_timer_->start(durration);
}

void QRosNode::spinRosSome()
{
  rclcpp::spin_some(node_ptr_);
}

QStringList QRosNode::getTopicsOfType(QString topic_type)
{
  QStringList topicsList;
  if (node_ptr_){
    const auto topics = node_ptr_->get_topic_names_and_types();
    for (const auto &topic : topics) {
      QString topicName = QString::fromStdString(topic.first);
      if(topic.second[0] == topic_type.toStdString()){
        topicsList.append(topicName);
      }
    }
  }
  return topicsList;
}

void QRosNode::declareParameter(const QString &param_name, const QVariant &default_value)
{
  auto param_value =  paramValueFromQVariant(default_value);
  node_ptr_->declare_parameter(param_name.toStdString(), param_value);
  updateParameters();
}


QVariantMap QRosNode::getParameters()
{

  //
  return parameters_;
}

void QRosNode::updateParameters()
{
  //QVariantMap param_map;
  std::map<std::string, rclcpp::Parameter> ros_map;
  node_ptr_->get_parameters("", ros_map); // Retrieve all parameters

  for (const auto &pair : ros_map) {
    const std::string &key = pair.first;
    const rclcpp::Parameter &param = pair.second;

    switch (param.get_type()) {
    case rclcpp::ParameterType::PARAMETER_BOOL:
      parameters_[QString::fromStdString(key)] = QVariant(param.get_value<bool>());
      break;
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      parameters_[QString::fromStdString(key)] = QVariant(qlonglong(param.get_value<int>()));
      break;
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      parameters_[QString::fromStdString(key)] = QVariant(param.get_value<double>());
      break;
    case rclcpp::ParameterType::PARAMETER_STRING:
      parameters_[QString::fromStdString(key)] = QVariant(QString::fromStdString(param.get_value<std::string>()));
      break;
    case rclcpp::ParameterType::PARAMETER_BYTE_ARRAY:
    case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY:
    case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
    case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
    case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
      // For arrays, convert each element and then push to a QVariantList
      QVariantList list;
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
        for (const auto &item : param.get_value<std::vector<int>>()) {
          list << QVariant(qlonglong(item));
        }
      } else if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
        for (const auto &item : param.get_value<std::vector<double>>()) {
          list << QVariant(item);
        }
      } else if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY) {
        for (const auto &item : param.get_value<std::vector<std::string>>()) {
          list << QVariant(QString::fromStdString(item));
        }
      } else if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL_ARRAY) {
        for (const auto &item : param.get_value<std::vector<bool>>()) {
          list << QVariant(item);
        }
      }
      parameters_[QString::fromStdString(key)] = list;
      break;
    }
  }
  emit parametersChanged();
}

void QRosNode::setParameter(const QString &param_name, const QVariant &value)
{
  auto param_value = paramValueFromQVariant(value);
  rclcpp::Parameter param(param_name.toStdString(),param_value);
  node_ptr_->set_parameter(param);
}

void QRosNode::setExternalParameterAsync(const QString &node_name,
                                 const QString &param_name,
                                 const QVariant &value,
                                 int wait_ms)
{
  std::thread t(&QRosNode::setExternalParameter, this, node_name, param_name, value, wait_ms);
  t.detach();
}

void QRosNode::setExternalParameter(const QString &node_name,
                            const QString &param_name,
                            const QVariant &value,
                            int wait_ms)
{
  auto param_client = std::make_shared<rclcpp::AsyncParametersClient>(node_ptr_, node_name.toStdString());

  if (param_client->wait_for_service(std::chrono::milliseconds(wait_ms))){
    auto param_value = paramValueFromQVariant(value);
    auto set_future = param_client->set_parameters({
        rclcpp::Parameter(param_name.toStdString(), param_value)
    });
    auto result = set_future.get()[0].successful;
    emit parameterSetResult(result,node_name,param_name);
  }else{
    emit parameterSetResult(false,node_name,param_name);
    RCLCPP_ERROR(node_ptr_->get_logger(), "Node %s not found, can't set parameter",node_name.toStdString().c_str());
  }

}

void QRosNode::getExternalParametersAsync(const QString &node_name, const QStringList &param_names, int wait_ms)
{
    std::thread t(&QRosNode::getExternalParameters, this, node_name, param_names, wait_ms);
    t.detach();
}

void QRosNode::getExternalParameters(const QString &node_name, const QStringList &param_names, int wait_ms)
{
    if (!node_ptr_) {
        RCLCPP_ERROR(node_ptr_->get_logger(), "Node pointer is null!");
        emit parametersGetResult(false, node_name, QVariantMap(), "Node pointer is null");
        return;
    }

    auto param_client = std::make_shared<rclcpp::AsyncParametersClient>(node_ptr_, node_name.toStdString());
    if (!param_client->wait_for_service(std::chrono::milliseconds(wait_ms))) {
        RCLCPP_WARN(node_ptr_->get_logger(), "Service not available for node: %s", node_name);
        emit parametersGetResult(false, node_name, QVariantMap(), "Service not available");
        return;
    }

    std::vector<std::string> std_param_names;
    for (const QString &name : param_names) {
        std_param_names.push_back(name.toStdString());
    }

    auto result = param_client->get_parameters(std_param_names);

    try {
        auto values = result.get();
        QVariantMap params;
        for (const auto &param : values) {
            params[QString::fromStdString(param.get_name())] = paramValueToQVariant(param);
        }
        emit parametersGetResult(true, node_name, params, "");
    } catch (const std::exception &e) {
        RCLCPP_ERROR(node_ptr_->get_logger(), "Exception fetching parameters: %s", e.what());
        emit parametersGetResult(false, node_name, QVariantMap(), QString::fromStdString(e.what()));
    }
}

void QRosNode::listExternalParametersAsync(const QString &node_name, int wait_ms) {
    std::thread([this, node_name, wait_ms]() {
        if (!node_ptr_) {
            emit parametersListResult(false, node_name, {}, "Node pointer is null!");
            return;
        }

        auto param_client = std::make_shared<rclcpp::AsyncParametersClient>(node_ptr_, node_name.toStdString());
        if (!param_client->wait_for_service(std::chrono::milliseconds(wait_ms))) {
            RCLCPP_WARN(node_ptr_->get_logger(), "Service not avalailable for node: %s", node_name);
            emit parametersListResult(false, node_name, {}, "Service not available for node: " + node_name);
            return;
        }

        auto list_future = param_client->list_parameters({}, 0);
        try {
            auto result = list_future.get();
            QStringList param_names;
            for (const auto& name : result.names) {
                param_names.append(QString::fromStdString(name));
            }
            emit parametersListResult(true, node_name, param_names, "");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_ptr_->get_logger(), "Exception listing parameters: %s", e.what());
            emit parametersListResult(false, node_name, {}, QString::fromStdString(e.what()));
        }
    }).detach();
}

QVariant QRosNode::paramValueToQVariant(const rclcpp::Parameter &param) {
    switch (param.get_type()) {
        case rclcpp::ParameterType::PARAMETER_BOOL:
            return QVariant(param.as_bool());
        case rclcpp::ParameterType::PARAMETER_INTEGER:
            return QVariant(qlonglong(param.as_int()));
        case rclcpp::ParameterType::PARAMETER_DOUBLE:
            return QVariant(param.as_double());
        case rclcpp::ParameterType::PARAMETER_STRING:
            return QVariant(QString::fromStdString(param.as_string()));
        case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY:
        case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
        case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
        case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
            return arrayToVariantList(param);
        case rclcpp::ParameterType::PARAMETER_NOT_SET:
            RCLCPP_ERROR(node_ptr_->get_logger(), "Parameter not set");
            break;
        default:
            RCLCPP_ERROR(node_ptr_->get_logger(), "Unhandled parameter array type: %s ",param.get_type_name().c_str());
            break;
    }
}

QVariant QRosNode::arrayToVariantList(const rclcpp::Parameter &param) {
    QVariantList list;
    switch (param.get_type()) {
    case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
        for (const auto &item : param.as_integer_array()) {
            list.append(QVariant(static_cast<qlonglong>(item))); 
        }
        break;
    case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
        for (const auto &item : param.as_double_array()) {
            list.append(QVariant(item));
        }
        break;
    case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
        for (const auto &item : param.as_string_array()) {
            list.append(QVariant(QString::fromStdString(item)));
        }
        break;
    case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY:
        for (const auto &item : param.as_bool_array()) {
            list.append(QVariant(item));
        }
        break;
    }
    return list;
}



QROS_NS_FOOT
