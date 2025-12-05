from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    planner = Node(
        package='planner',
        executable='planner',   
        name='planner',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': True}]
    )

    actuator = Node(
        package='actuator',
        executable='actuator',  
        name='actuator',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        planner,
        actuator
    ])
