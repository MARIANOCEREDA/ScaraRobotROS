#define NODE_NAME "inverse_kinematics_node"
#include "inverse_kinematics_node.hpp"

#define WS_TOPIC_NAME "/ws_position"

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

void InverseKinematicsNode::CalculateInverseKinematics(float x, float y, float z)
{
    
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