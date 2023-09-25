import os
import xacro
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    ##Find any files

    pkg_path = FindPackageShare("my_bot").find("my_bot")

    # Find the XACRO and URDF for the Robot
    xacro_path = os.path.join(pkg_path, "description", "robot.urdf.xacro")
    robot_description_config = xacro.process_file(xacro_path).toxml()

    # Find the rviz_config file
    rviz_config_path = os.path.join(pkg_path, "config/view_bot.rviz")    

    ##Launch Configuration
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz_config_file = LaunchConfiguration("rviz_config_file")


    ## Declarations

    declare_use_sim_time = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=rviz_config_path,
        description='Full path to the RVIZ config file to use')


    ##Nodes

    #Launch robot_state_publisher
    params = {
        "robot_description": robot_description_config,
        "use_sim_time": use_sim_time,
    }
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[params],
    )
    
    #Launch Joint_State_pub
    node_joint_state_pub= Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
    )

    #Launch RVIZ
    node_start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )


    ld = LaunchDescription()
    ## Declare Launch Options
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_rviz_config_file_cmd)


    ##Add Nodes
    ld.add_action(node_robot_state_publisher)
    # ld.add_action(node_joint_state_pub)
    ld.add_action(node_start_rviz_cmd)
    return ld
