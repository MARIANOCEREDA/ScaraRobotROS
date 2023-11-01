#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/float64_multi_array.hpp"

#define NODE_NAME "inverse_kinematics_node"

class InverseKinematicsNode : public rclcpp::Node
{
    private:
        rclcpp::Publisher<example_interfaces::msg::Float64MultiArray>::SharedPtr pub_joint;
        rclcpp::Subscription<example_interfaces::msg::Float64MultiArray>::SharedPtr ws_sub_joint;

    public:
        InverseKinematicsNode();
        void WsCallback(const example_interfaces::msg::Float64MultiArray::SharedPtr position);
        void PublishJoints();
        void CalculateInverseKinematics(float x, float y, float z);
};