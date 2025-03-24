from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.substitutions import PathJoinSubstitution, TextSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro
import os

def generate_launch_description():

    baud_rate_launch_arg = DeclareLaunchArgument(
      'baud_rate', default_value=TextSubstitution(text='9600')
    )
    device_launch_arg = DeclareLaunchArgument(
      'device', default_value=TextSubstitution(text='COM1')
    )
    simulation_launch_arg = DeclareLaunchArgument(
      'simulation', default_value=TextSubstitution(text='false')
    )

    # Les xacro fil og viderekoble launch parametere, funksjon hentet og tilpasset fra https://robotics.stackexchange.com/questions/102165/ros2-python-launch-how-to-forward-launch-arguments-to-xacro-models
    def create_robot_description(context): 
        xacro_file = os.path.join(get_package_share_directory("qube_bringup"),"urdf","controlled_qube.urdf.xacro")    
        assert os.path.exists(xacro_file), "The .xacro doesnt exist in "+str(xacro_file)
        robot_description_config = xacro.process_file(xacro_file, 
            mappings={  "baud_rate": context.launch_configurations['baud_rate'], 
                        "device": context.launch_configurations['device'],
                        "simulation" : context.launch_configurations['simulation']})
        robot_desc = robot_description_config.toxml()
        return [SetLaunchConfiguration('robot_desc', robot_desc)]

    create_robot_description_arg = OpaqueFunction(function=create_robot_description)

    return LaunchDescription([
        baud_rate_launch_arg,
        device_launch_arg,
        simulation_launch_arg,
        create_robot_description_arg,
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
            parameters=[{'robot_description': LaunchConfiguration('robot_desc')}]
        ),
        # Kjør joint state publisher gui
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui'
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
