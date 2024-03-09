from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description(): 
 
 # Get URDF via xacro
 
 	
    description_package = "iiwa_description"
    description_file = "iiwa_dual.config.xacro"
    
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare(description_package), 'config', description_file]
            ),
           
        ]
    )

    robot_description = {'robot_description': robot_description_content}
    
    node = Node(
        package='robot_commander',
        executable='test_urdf',
        parameters=[robot_description],
        output='both',
        
    )
    
    return LaunchDescription([node])
