import os
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # Hent controlled qube xacro fil
    xacro_file = os.path.join(get_package_share_directory("qube_bringup"),"urdf","controlled_qube.urdf.xacro")
    robot_description_content = xacro.process_file(xacro_file).toxml()

    return LaunchDescription([
        # Kjør rviz2 med lagret konfigurasjonsfil.
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', [os.path.join(get_package_share_directory("qube_bringup"), 'config', 'rviz_parameters.rviz')]]
        ),
        # Kjør robot state publisher med qube
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}]
        ),
        # Kjør launch fil fra qube_driver pakke
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('qube_driver'),
                    'launch',
                    'qube_driver.launch.py'
                ])
            ])
        )
    ])
