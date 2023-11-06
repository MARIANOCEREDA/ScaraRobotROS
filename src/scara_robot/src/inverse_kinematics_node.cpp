#define NODE_NAME "inverse_kinematics_node"
#include "inverse_kinematics_node.hpp"
#include "string.h"

#define WS_TOPIC_NAME "/robot_position"
#define JOINT_STATE_TOPIC_NAME "/set_joint_trajectory"

InverseKinematicsNode::InverseKinematicsNode() : Node(NODE_NAME) {

    std::string info_msg = "Node " + std::string(NODE_NAME) + " created.";
    RCLCPP_INFO(this->get_logger(), info_msg.c_str());

    ws_sub_joint = this->create_subscription<example_interfaces::msg::Float64MultiArray>(
        WS_TOPIC_NAME, 
        10,
        std::bind(&InverseKinematicsNode::WsCallback, this, std::placeholders::_1));
    
    pub_joint_state = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        JOINT_STATE_TOPIC_NAME,
        10
    );
}

void InverseKinematicsNode::WsCallback(const example_interfaces::msg::Float64MultiArray::SharedPtr position)
{
    float x = position->data[0];
    float y = position->data[1];
    float z = position->data[2];
    std::string info_msg = "Incomming point: {" + std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(z) + "}";
    RCLCPP_INFO(this->get_logger(), info_msg.c_str());
    CalculateInverseKinematics(x, y, z);
}

void InverseKinematicsNode::CalculateInverseKinematics(const float& px,const float& py,const float& pz)
{
    float num, den, beta, alpha;

    q[0] = pz;

    num = std::pow(px, 2) + std::pow(py, 2) - std::pow(robot_dim.a1, 2) - std::pow(robot_dim.a2, 2);
    den = 2 * robot_dim.a1 * robot_dim.a2;

    q[2] = std::acos(num / den);

    num = robot_dim.a2 * std::sin(q[2]);
    den = robot_dim.a1 + robot_dim.a2 * std::cos(q[2]);
    alpha = std::atan(num / den);

    beta = std::atan(px / py); 

    q[1] = beta - alpha;

    std::string info_msg = "Angles[rad]: {" + std::to_string(q[0]) + "," + std::to_string(q[1]) + "," + std::to_string(q[2]) + "}";
    RCLCPP_INFO(this->get_logger(), info_msg.c_str());

    PublishJoints();
}

void InverseKinematicsNode::PublishJoints()
{
    RCLCPP_INFO(this->get_logger(), "Publishing joints ...");

    auto joint_trajectory_msg = trajectory_msgs::msg::JointTrajectory();
    joint_trajectory_msg.header.stamp.sec = 0;
    joint_trajectory_msg.header.stamp.nanosec = 0;
    joint_trajectory_msg.header.frame_id = "base_link";

    joint_trajectory_msg.joint_names.push_back("joint0");
    joint_trajectory_msg.joint_names.push_back("joint1");
    joint_trajectory_msg.joint_names.push_back("joint2");
    
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.push_back(q[0]); // Posición
    point.positions.push_back(q[1]); // Posición
    point.positions.push_back(q[2]); // Posición

    point.time_from_start.sec = 1;   // Tiempo en segundos
    point.time_from_start.nanosec = 0;

    joint_trajectory_msg.points.push_back(point);

    pub_joint_state->publish(joint_trajectory_msg);
}


int main(int arg, char **argv)
{
    rclcpp::init(arg, argv);
    auto node = std::make_shared<InverseKinematicsNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}