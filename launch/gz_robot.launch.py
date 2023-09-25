import os
from os.path import join
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, LogInfo
from ament_index_python import get_package_share_directory

def generate_launch_description():

    ## Paths to Files
    
    pkg_path = FindPackageShare('my_bot').find('my_bot')
    gazebo_path = FindPackageShare('ros_gz_sim').find('ros_gz_sim')
    # world_path = os.path.join(pkg_path, 'worlds', 'empty.sdf')

    # world_file = LaunchConfiguration("world_file", default = world_path)


    bcr_bot_path = get_package_share_directory("my_bot")
    world_file = LaunchConfiguration("world_file", default = join(bcr_bot_path, "worlds", "empty.sdf"))


    ## Launch rsp.launch

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            pkg_path, 'launch', 'rsp.launch.py')]), 
            launch_arguments={'use_sim_time':'true'}.items()
    )

    gz_sim_share = get_package_share_directory("ros_gz_sim")
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(gz_sim_share, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args" : world_file
        }.items()
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'my_bot'],
        output='screen'
    )


    return LaunchDescription([
        DeclareLaunchArgument("world_file", default_value=world_file),
        rsp,
        gz_sim,
        spawn_entity
    ])
    # ld = LaunchDescription()

    # ld.add_action(world)
    # ld.add_action(rsp)
    # ld.add_action(gazebo)
    # ld.add_action(spawn_entity)

    # return ld