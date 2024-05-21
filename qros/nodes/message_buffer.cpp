#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include <vector>
#include <string>
#include <map>

class LatchedPublisherNode : public rclcpp::Node
{
public:
  LatchedPublisherNode()
      : Node("latched_publisher_node")
  {
    // Declare and get parameters
    this->declare_parameter<std::vector<std::string>>("input_topics", std::vector<std::string>());
    this->declare_parameter<std::vector<std::string>>("output_topics", std::vector<std::string>());
    this->declare_parameter<std::vector<std::string>>("message_types", std::vector<std::string>());

    auto input_topics = this->get_parameter("input_topics").as_string_array();
    auto output_topics = this->get_parameter("output_topics").as_string_array();
    auto message_types = this->get_parameter("message_types").as_string_array();

    if (input_topics.size() != output_topics.size() || input_topics.size() != message_types.size()) {
      RCLCPP_ERROR(this->get_logger(), "The number of input topics, output topics, and message types must match.");
      rclcpp::shutdown();
      return;
    }

    for (size_t i = 0; i < input_topics.size(); ++i) {
      create_subscription_and_publisher(input_topics[i], output_topics[i], message_types[i]);
    }
  }

private:
  void create_subscription_and_publisher(const std::string &input_topic, const std::string &output_topic, const std::string &message_type)
  {
    // Create generic subscriber
    auto subscription = this->create_generic_subscription(
        input_topic, message_type, rclcpp::SystemDefaultsQoS(),
        [this, output_topic, message_type](std::shared_ptr<rclcpp::SerializedMessage> msg) {
          this->topic_callback(output_topic, message_type, msg);
        });

    subscriptions_.push_back(subscription);

    // Output topic publishers will be created on demand in the callback
    RCLCPP_INFO(this->get_logger(), "Subscribed to topic: %s with message type: %s", input_topic.c_str(), message_type.c_str());
    RCLCPP_INFO(this->get_logger(), "Will publish to latched topic: %s", output_topic.c_str());
  }

  void topic_callback(const std::string &output_topic, const std::string &message_type, std::shared_ptr<rclcpp::SerializedMessage> msg)
  {
    if (publishers_.find(output_topic) == publishers_.end()) {
      // Create generic publisher with latched QoS if not already created
      auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
      auto publisher = this->create_generic_publisher(output_topic, message_type, qos);
      publishers_[output_topic] = publisher;
    }
    RCLCPP_INFO(this->get_logger(), "Received message of size: '%zu' bytes for topic: '%s'", msg->size(), output_topic.c_str());
    publishers_[output_topic]->publish(*msg);
    RCLCPP_INFO(this->get_logger(), "Published message of size: '%zu' bytes on topic: '%s'", msg->size(), output_topic.c_str());
  }

  std::vector<rclcpp::GenericSubscription::SharedPtr> subscriptions_;
  std::map<std::string, rclcpp::GenericPublisher::SharedPtr> publishers_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LatchedPublisherNode>());
  rclcpp::shutdown();
  return 0;
}
