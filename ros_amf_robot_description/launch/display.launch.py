from launch_ros.actions import Node
from launch import LaunchDescription
import xacro
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    share_dir = get_package_share_directory('ros_amf_robot_description')
    xacro_file = os.path.join(share_dir, 'urdf', 'ros_amf_robot.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()
    rviz_config_file = os.path.join(share_dir, 'config', 'display.rviz')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf}
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        rviz_node,
    ])