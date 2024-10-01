#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class HelloSubscriberNode : public rclcpp::Node
{
public:
    HelloSubscriberNode() : Node("hello_subscriber") // Node name
    {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "hello_topic", 10, std::bind(&HelloSubscriberNode::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HelloSubscriberNode>(); // Create subscriber node
    rclcpp::spin(node); // Keep the node alive to listen for messages
    rclcpp::shutdown();
    return 0;
}
