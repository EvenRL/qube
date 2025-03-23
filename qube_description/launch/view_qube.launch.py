import os
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Hent qube xacro fil
    xacro_file = os.path.join(get_package_share_directory("qube_description"),"urdf","qube.urdf.xacro")
    robot_description_content = xacro.process_file(xacro_file).toxml()

    return LaunchDescription([
        # Kjør rviz2 med lagret konfigurasjonsfil.
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', [os.path.join(get_package_share_directory("qube_description"), 'config', 'rviz_parameters.rviz')]]
        ),
        # Kjør robot state publisher med qube
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}]
        ),
        # Kjør joint state publisher gui
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui'
        )
    ])
