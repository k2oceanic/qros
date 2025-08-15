#pragma once

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

class ParameterProxyNode : public rclcpp::Node
{
public:
  ParameterProxyNode();

private:
  void fetch_and_bind(const std::string &target_ns);

  void bind_param(const std::string &target_ns, const std::string &param_name, const rcl_interfaces::msg::ParameterValue &value);

  void set_param(const std::string &target_ns, const rcl_interfaces::msg::Parameter &param);

  rclcpp::Client<rcl_interfaces::srv::ListParameters>::SharedPtr list_client_;
  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr get_client_;
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr set_client_;

  std::unordered_map<std::string, rclcpp::PublisherBase::SharedPtr> publishers_;
  std::unordered_map<std::string, rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  std::unordered_map<std::string, rcl_interfaces::msg::ParameterValue> current_values_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  double publish_rate_;
};
