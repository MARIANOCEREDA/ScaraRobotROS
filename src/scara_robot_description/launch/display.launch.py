from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_path

PACKAGE_NAME = "scara_robot_description"

def generate_launch_description():

    urdf_path = os.path.join(get_package_share_path(PACKAGE_NAME), 
                             'urdf', 'scara.urdf.xacro')
    
    robot_description_param = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    rviz_config_path = os.path.join(get_package_share_path(PACKAGE_NAME),
                             'rviz', 'config.rviz')
    
    node_robot_state_publisher = Node(
        executable="robot_state_publisher",
        package="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description_param}
        ]   
    )

    node_joint_states_pub_gui = Node(
        executable="joint_state_publisher_gui",
        package="joint_state_publisher_gui"
    )

    node_rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_path]
    )

    return LaunchDescription([
        node_robot_state_publisher,
        node_joint_states_pub_gui,
        node_rviz
    ])