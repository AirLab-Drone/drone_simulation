#
#
# to this first
# 
# export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ros2_ws_sim/src/AirLab_in_gazebo/models
#


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_AirLab_in_gazebo = get_package_share_directory('AirLab_in_gazebo')
    launch_file_dir = os.path.join(get_package_share_directory('AirLab_in_gazebo'), 'launch')


    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'pause': 'true'}.items()
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'world',
            default_value=[os.path.join(pkg_AirLab_in_gazebo, 'worlds', 'empty_world.world'), ''],
            description='SDF world file',
        ),
        gazebo,
        # turtlebot
    ])