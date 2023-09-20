"""
    This file is here for reference and comments only to help understand ros2 launch files. It is not intended to be used as a finished launch file
    Another file which is cleaner is used instead
"""

import os
import xacro

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from ament_index_python import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    """DeclareLaunchArguments 
            allow the user to input arguments to change aspects of the launch file. This could be changing the background colour of a sim, using sim time or not etc...
            To see all arguments which can be used, type "<ros2 launch package_name <launch_file_name>.launch.py -s>"
    """


    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')
    


    ##
    """The following code does the same thing except FindPackageShare does not require to use os.path.join 
            There does not seem to be a preferred standard to use
            FindPackageShare calles get_package_share_directory"""

    pkg_path = FindPackageShare('my_bot').find('my_bot')
    print(pkg_path)
    # pkg_path = os.path.join(get_package_share_directory('my_bot'))

    ##

    # Find the XACRO and URDF for the Robot
    xacro_path = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_path).toxml()

    rviz_config_path = os.path.join(pkg_path, 'config/view_bot.rviz')

    # rviz_config_file = LaunchConfiguration('rviz_config_file')

    # declare_rviz_config_file_cmd = DeclareLaunchArgument(
    #     name='rviz_config_file',
    #     default_value=rviz_config_path,
    #     description='Full path to the RVIZ config file to use')

    # start_rviz_cmd = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     arguments=['-d', rviz_config_file])

    
    
    params = {'robot_description': robot_description_config,
              'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # rviz_params = {'view_bot.rviz': LaunchConfiguration('rviz_config_file')}

    # node_rviz = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     output='screen',
    #     parameters=[rviz_params]
    # )

    # node_joint_state_pub= Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     output='screen',
    # )



    # sim_time = DeclareLaunchArgument(
    #     'use_sim_time',
    #     default_value='false',
    #     description='Use sim time if true'),

    # ld.add_action(new_background_r_launch_arg)

    """THE ORDER MATTERS 
        You need to make sure that any Launch configurations which have been used in a node must be declared before adding the node
        Here, node_robot_state_publisher uses use_sim_time as a launch condition. 
            ***THis means that the delcare_use_sim_time MUST be first***"""
    
    ld.add_action(declare_use_sim_time)
    ld.add_action(node_robot_state_publisher)
    ld.add_entity(declare_use_sim_time)

    return ld

    # return LaunchDescription([
        

    #     new_background_r_launch_arg,
    #     node_robot_state_publisher
    #     # node_robot_state_publisher,  use_sim_time
    #     # declare_rviz_config_file_cmd,
    #     # start_rviz_cmd, node_joint_state_pub,
    # ])
