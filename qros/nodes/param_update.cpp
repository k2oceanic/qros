#include <rclcpp/rclcpp.hpp>
#include <memory>

class ParameterListenerNode : public rclcpp::Node
{
public:
    ParameterListenerNode()
    : Node("parameter_listener_node")
    {
        // Declare and initialize a parameter
        this->declare_parameter<std::string>("my_parameter", "default_value");

        // Set a callback to handle changes to parameters
        parameter_event_sub_ = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> &parameters) {
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;

                for (const auto &parameter : parameters) {
                    RCLCPP_INFO(this->get_logger(), "Parameter '%s' changed to '%s'",
                                parameter.get_name().c_str(),
                                parameter.value_to_string().c_str());
                }
                return result;
            }
        );
    }

private:
    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr parameter_event_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ParameterListenerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}   
