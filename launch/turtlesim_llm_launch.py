from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),


        Node(
            package='my_turtle_controller',
            executable='smart_turtle',
            name='SmartTurtle',
            output='screen',
            emulate_tty=True,
            parameters=[],
            prefix='gnome-terminal --'
        )
    ])