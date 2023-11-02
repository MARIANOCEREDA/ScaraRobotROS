#define NODE_NAME "inverse_kinematics_node"
#include "inverse_kinematics_node.hpp"
#include "string.h"

#define WS_TOPIC_NAME "/position"


InverseKinematicsNode::InverseKinematicsNode() : Node(NODE_NAME) {

    std::string info_msg = "Node " + std::string(NODE_NAME) + " created.";
    RCLCPP_INFO(this->get_logger(), info_msg.c_str());

    ws_sub_joint = this->create_subscription<example_interfaces::msg::Float64MultiArray>(
        WS_TOPIC_NAME, 
        10,
        std::bind(&InverseKinematicsNode::WsCallback, this, std::placeholders::_1));
}

void InverseKinematicsNode::WsCallback(const example_interfaces::msg::Float64MultiArray::SharedPtr position)
{
    float x = position->data[0];
    float y = position->data[1];
    float z = position->data[2];
    CalculateInverseKinematics(x, y, z);
}

void InverseKinematicsNode::CalculateInverseKinematics(float px, float py, float pz)
{
    float num, den, beta, alpha;

    q1 = pz;

    num = pow(px, 2) + pow(py, 2) - pow(robot_dim.a1, 2) + pow(robot_dim.a2, 2);
    den = 2 * robot_dim.a1 * robot_dim.a2;

    q3 = num / den;

    num = robot_dim.a2 * std::sin(q3);
    den = robot_dim.a1 + robot_dim.a2 * std::cos(q3);
    alpha = std::acos(num) / (den);

    beta = std::atan(px / (pow(px,2) + pow(py,2))); 

    q2 = beta - alpha;

    std::string info_msg = "{" + std::to_string(q1) + "," + std::to_string(q2) + "," + std::to_string(q1);
    RCLCPP_INFO(this->get_logger(), info_msg.c_str());
}

void InverseKinematicsNode::PublishJoints()
{

}


int main(int arg, char **argv)
{
    rclcpp::init(arg, argv);
    auto node = std::make_shared<InverseKinematicsNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}