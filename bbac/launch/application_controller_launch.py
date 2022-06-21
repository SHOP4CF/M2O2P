from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    ac_node = Node(
        package="ac",
        executable="application_controller",
        output='screen',
        emulate_tty=True,
        arguments=[('__log_level:=debug')]
    )
    
    ld.add_action(ac_node)
    
    return ld  
        