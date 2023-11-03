#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/float64_multi_array.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "robot_dimensions.hpp"
#include <cmath>

#define NODE_NAME "inverse_kinematics_node"

class InverseKinematicsNode : public rclcpp::Node
{
    private:
        rclcpp::Publisher<example_interfaces::msg::Float64MultiArray>::SharedPtr pub_joint;
        rclcpp::Subscription<example_interfaces::msg::Float64MultiArray>::SharedPtr ws_sub_joint;
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_joint_state;
        Robot::robot_dimensions robot_dim;
        float q[3] = {0.0, 0.0, 0.0};

    public:
        InverseKinematicsNode();
        void WsCallback(const example_interfaces::msg::Float64MultiArray::SharedPtr position);
        void PublishJoints();
        void CalculateInverseKinematics(const float&,const float&,const float&);
};