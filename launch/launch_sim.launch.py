import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    ## Paths to Files
    
    pkg_path = FindPackageShare('my_bot').find('my_bot')
    gazebo_path = FindPackageShare('gazebo_ros').find('gazebo_ros')
    ## Launch rsp.launch
    xacro_path = os.path.join(pkg_path, "description", "robot.urdf.xacro")
    # world_path = os.path.join(pkg_path, 'worlds', 'NuBot_World', 
    #                           'terrain-lab_room', 'model.sdf')
    world_path = os.path.join(pkg_path, 'worlds', 'new.world') 


    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            pkg_path, 'launch', 'rsp.launch.py')]), 
            launch_arguments={'use_sim_time':'true'}.items()
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            gazebo_path, 'launch', 'gazebo.launch.py'
        )])
    )


    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 
                   'robot_description', '-entity', 'my_bot'],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(rsp)
    ld.add_action(gazebo)
    ld.add_action(spawn_entity)

    return ld