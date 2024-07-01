#include <chrono>
#include <functional>
#include <memory>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class JointStatePublisher : public rclcpp::Node
{
public:
    JointStatePublisher()
    : Node("joint_state_publisher"), angle_(0.0)
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        timer_ = this->create_wall_timer(100ms, std::bind(&JointStatePublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto message = sensor_msgs::msg::JointState();
        message.header.stamp = this->now();
        message.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
        angle_ += 0.1;
        message.position = {
            std::sin(angle_), 
            std::cos(angle_), 
            std::sin(angle_), 
            std::cos(angle_), 
            std::sin(angle_), 
            std::cos(angle_),
            std::sin(angle_)
        };
        publisher_->publish(message);
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    double angle_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointStatePublisher>());
    rclcpp::shutdown();
    return 0;
}
