
'''
Change your work space name
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ros2_ws/src/AirLab_in_gazebo/models
ros2 launch AirLab_in_gazebo launch_sim.launch.py world:=./src/AirLab_in_gazebo/worlds/my_house.world 
'''

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node



def generate_launch_description():

    robot_name = LaunchConfiguration('robot_name')
    height = LaunchConfiguration('height')
    use_sim_time = LaunchConfiguration('use_sim_time')

    package_name='AirLab_in_gazebo'

    pkg_path = os.path.join(get_package_share_directory(package_name))
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')
    

    robot_description_config = Command([
        'xacro ', xacro_file
        ])

    params = {'robot_description': robot_description_config}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )


    world = os.path.join(
        get_package_share_directory('AirLab_in_gazebo'),
        'worlds',
        'my_house.world'
    )


    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'configs','gazebo_params.yaml')

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={
                        'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file,
                        'world': world
                    }.items()
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', robot_name,
                                   '-z', height],
                        output='screen')

    # rviz2
    rviz =  Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(LaunchConfiguration("rviz")),
        arguments=['-d' + os.path.join(get_package_share_directory(package_name), 'configs', 'sim.rviz')]
        )



    # Launch them all!
    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true'),

        DeclareLaunchArgument(
            'robot_name', default_value='airlab_drone',
            description='The name of robot'),

        DeclareLaunchArgument(
            'height', default_value='0.13',
            description='The height of robot generate'),

        DeclareLaunchArgument(
            'rviz', default_value='false', 
            description='Launch RVIZ (optional).'),
        
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        rviz,


    ])