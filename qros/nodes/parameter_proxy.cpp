#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/srv/list_parameters.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

class ParameterProxyNode : public rclcpp::Node {
public:
  ParameterProxyNode() : Node("parameter_proxy_node") {
    this->declare_parameter("target_node", "");
    this->declare_parameter("state_publish_rate", 1.0);
    std::string target_node = this->get_parameter("target_node").as_string();
    if (target_node.empty()) {
      RCLCPP_FATAL(this->get_logger(), "Missing required parameter: target_node");
      rclcpp::shutdown();
      return;
    }

    publish_rate_ = this->get_parameter("state_publish_rate").as_double();
    RCLCPP_INFO(this->get_logger(), "Target node: %s", target_node.c_str());

    list_client_ = this->create_client<rcl_interfaces::srv::ListParameters>(target_node + "/list_parameters");
    get_client_ = this->create_client<rcl_interfaces::srv::GetParameters>(target_node + "/get_parameters");
    set_client_ = this->create_client<rcl_interfaces::srv::SetParameters>(target_node + "/set_parameters");

    // Wait for service
    while (!list_client_->wait_for_service(1s)) {
      RCLCPP_INFO(this->get_logger(), "Waiting for %s to be available...", list_client_->get_service_name());
    }

    fetch_and_bind(target_node);

    publish_timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / publish_rate_), [this]() {
      for (const auto &entry : publishers_) {
        const std::string &param_name = entry.first;
        const auto &pub = entry.second;
        const auto &val = current_values_[param_name];

        switch (val.type) {
        case rcl_interfaces::msg::ParameterType::PARAMETER_STRING: {
          auto typed_pub = std::dynamic_pointer_cast<rclcpp::Publisher<std_msgs::msg::String>>(pub);
          if (typed_pub) {
            std_msgs::msg::String msg;
            msg.data = val.string_value;
            typed_pub->publish(msg);
          }
          break;
        }
        case rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER: {
          auto typed_pub = std::dynamic_pointer_cast<rclcpp::Publisher<std_msgs::msg::Int64>>(pub);
          if (typed_pub) {
            std_msgs::msg::Int64 msg;
            msg.data = val.integer_value;
            typed_pub->publish(msg);
          }
          break;
        }
        case rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE: {
          auto typed_pub = std::dynamic_pointer_cast<rclcpp::Publisher<std_msgs::msg::Float64>>(pub);
          if (typed_pub) {
            std_msgs::msg::Float64 msg;
            msg.data = val.double_value;
            typed_pub->publish(msg);
          }
          break;
        }
        case rcl_interfaces::msg::ParameterType::PARAMETER_BOOL: {
          auto typed_pub = std::dynamic_pointer_cast<rclcpp::Publisher<std_msgs::msg::Bool>>(pub);
          if (typed_pub) {
            std_msgs::msg::Bool msg;
            msg.data = val.bool_value;
            typed_pub->publish(msg);
          }
          break;
        }
        }
      }
    });
  }

private:
  rclcpp::Client<rcl_interfaces::srv::ListParameters>::SharedPtr list_client_;
  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr get_client_;
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr set_client_;

  std::unordered_map<std::string, rclcpp::PublisherBase::SharedPtr> publishers_;
  std::unordered_map<std::string, rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  std::unordered_map<std::string, rcl_interfaces::msg::ParameterValue> current_values_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  double publish_rate_;

  void fetch_and_bind(const std::string &target_ns) {
    auto list_req = std::make_shared<rcl_interfaces::srv::ListParameters::Request>();
    auto list_future = list_client_->async_send_request(list_req);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), list_future) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to call list_parameters");
      return;
    }
    auto list_resp = list_future.get();

    const auto &param_names = list_resp->result.names;
    if (param_names.empty()) {
      RCLCPP_WARN(this->get_logger(), "No parameters declared in target node: %s", target_ns.c_str());
      return;
    }

    auto get_req = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    get_req->names = param_names;
    auto get_future = get_client_->async_send_request(get_req);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), get_future) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to call get_parameters");
      return;
    }
    auto get_resp = get_future.get();

    const auto &params = get_resp->values;
    for (size_t i = 0; i < param_names.size(); ++i) {
      current_values_[param_names[i]] = params[i];
      bind_param(target_ns, param_names[i], params[i]);
    }
  }

  void bind_param(const std::string &target_ns, const std::string &param_name, const rcl_interfaces::msg::ParameterValue &value) {
    std::string topic_name = param_name;
    std::replace(topic_name.begin(), topic_name.end(), '.', '/');
    while (topic_name.find("//") != std::string::npos) {
      topic_name.erase(topic_name.find("//"), 1);
    }
    if (!topic_name.empty() && topic_name.front() != '/') topic_name = "/" + topic_name;
    std::string base_topic = target_ns + topic_name;

    switch (value.type) {
    case rcl_interfaces::msg::ParameterType::PARAMETER_STRING: {
      auto pub = this->create_publisher<std_msgs::msg::String>(base_topic + "/state", 10);
      publishers_[param_name] = pub;
      auto sub = this->create_subscription<std_msgs::msg::String>(base_topic + "/set", 10,
                                                                  [this, param_name, target_ns](const std_msgs::msg::String::SharedPtr msg) {
                                                                    rcl_interfaces::msg::Parameter p;
                                                                    p.name = param_name;
                                                                    p.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
                                                                    p.value.string_value = msg->data;
                                                                    current_values_[param_name] = p.value;
                                                                    set_param(target_ns, p);
                                                                  });
      subscriptions_[param_name] = sub;
      break;
    }
    case rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER: {
      auto pub = this->create_publisher<std_msgs::msg::Int64>(base_topic + "/state", 10);
      publishers_[param_name] = pub;
      auto sub = this->create_subscription<std_msgs::msg::Int64>(base_topic + "/set", 10,
                                                                 [this, param_name, target_ns](const std_msgs::msg::Int64::SharedPtr msg) {
                                                                   rcl_interfaces::msg::Parameter p;
                                                                   p.name = param_name;
                                                                   p.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
                                                                   p.value.integer_value = msg->data;
                                                                   current_values_[param_name] = p.value;
                                                                   set_param(target_ns, p);
                                                                 });
      subscriptions_[param_name] = sub;
      break;
    }
    case rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE: {
      auto pub = this->create_publisher<std_msgs::msg::Float64>(base_topic + "/state", 10);
      publishers_[param_name] = pub;
      auto sub = this->create_subscription<std_msgs::msg::Float64>(base_topic + "/set", 10,
                                                                   [this, param_name, target_ns](const std_msgs::msg::Float64::SharedPtr msg) {
                                                                     rcl_interfaces::msg::Parameter p;
                                                                     p.name = param_name;
                                                                     p.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
                                                                     p.value.double_value = msg->data;
                                                                     current_values_[param_name] = p.value;
                                                                     set_param(target_ns, p);
                                                                   });
      subscriptions_[param_name] = sub;
      break;
    }
    case rcl_interfaces::msg::ParameterType::PARAMETER_BOOL: {
      auto pub = this->create_publisher<std_msgs::msg::Bool>(base_topic + "/state", 10);
      publishers_[param_name] = pub;
      auto sub = this->create_subscription<std_msgs::msg::Bool>(base_topic + "/set", 10,
                                                                [this, param_name, target_ns](const std_msgs::msg::Bool::SharedPtr msg) {
                                                                  rcl_interfaces::msg::Parameter p;
                                                                  p.name = param_name;
                                                                  p.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
                                                                  p.value.bool_value = msg->data;
                                                                  current_values_[param_name] = p.value;
                                                                  set_param(target_ns, p);
                                                                });
      subscriptions_[param_name] = sub;
      break;
    }
    default:
      RCLCPP_WARN(this->get_logger(), "Skipping unsupported or unset parameter '%s' (type: %d)", param_name.c_str(), value.type);
      break;
    }
  }

  void set_param(const std::string &target_ns, const rcl_interfaces::msg::Parameter &param) {
    auto req = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    req->parameters.push_back(param);
    set_client_->async_send_request(req);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ParameterProxyNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
