#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class RobotDescriptionSubscriber : public rclcpp::Node
{
public:
    RobotDescriptionSubscriber()
    : Node("robot_description_subscriber")
    {
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
        qos.durability(rclcpp::DurabilityPolicy::TransientLocal);

        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/robot_description", qos, std::bind(&RobotDescriptionSubscriber::listener_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<std_msgs::msg::String>("controller_manager/robot_description", qos);
  }

private:
    void listener_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        this->declare_parameter("robot_description", msg->data);
        publisher_->publish(*msg);
    }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotDescriptionSubscriber>());
    rclcpp::shutdown();
    return 0;
}