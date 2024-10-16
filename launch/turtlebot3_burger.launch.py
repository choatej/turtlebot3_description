import logging
import os
import xacro
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, SetEnvironmentVariable, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


package_name = "turtlebot3_description"
robot_name = "turtlebot3"


log_level = os.environ.get('LOG_LEVEL', 'INFO').upper()
logging.basicConfig(level=log_level)
logging.info('logging configured')
logger = logging.getLogger(__name__)


def validate_teleop_type(teleop_type_val: str) -> None:
    if teleop_type_val not in ['keyboard', 'joystick', 'none']:
        raise ValueError('Invalid teleop_type. Must be one of: keyboard, joystick, none')


start_gazebo = LaunchConfiguration('start_gazebo')
start_rviz = LaunchConfiguration('start_rviz')
teleop_type = LaunchConfiguration('teleop_type')
world_file = LaunchConfiguration('world_file')


def start_gazebo_arg():
    logger.debug('adding start_gazebo arg')
    return DeclareLaunchArgument(
        'start_gazebo',
        default_value='true',
        description='Launch Gazebo simulation'
    )


def start_rviz_arg():
    logger.debug('adding start_rviz arg')
    return DeclareLaunchArgument(
        'start_rviz',
        default_value='false',
        description='Launch RViz'
    )


def teleop_type_arg():
    logger.debug('adding teleop_type arg')
    return DeclareLaunchArgument(
        'teleop_type',
        default_value='none',
        description='Start teleop node. Values are keyboard, joystick, or none'
    )


def world_file_arg():
    logger.debug('adding world file arg')
    return DeclareLaunchArgument(
        'world_file',
        default_value='worlds/tenBySixteenRoom.world',
        description='Path to the Gazebo world file'
    )


def robot_state_publisher():
    logger.debug('adding robot state publisher node')
    xacro_file_path = os.path.join(get_package_prefix(package_name), 'share', package_name, 'urdf', 'turtlebot3_burger.urdf.xacro')
    urdf_text = xacro.process_doc(xacro.parse(xacro_file_path)).toprettyxml(indent='    ')
    return Node(package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher',
                parameters=[{'use_sim_time': True, 'robot_description': urdf_text}], output='screen')


def robot_spawn():
    logger.debug('adding spawn entity node')
    return Node(name='spawn_entity', package='gazebo_ros', executable='spawn_entity.py',
                arguments=['-entity', robot_name, '-x', '0.0', '-y', '0.0', '-z', '0.0', '-topic', 'robot_description'],
                parameters=[{'use_sim_time': True}], output='screen')


def static_transform_publisher():
    logger.debug('adding static transform publisher node')
    return Node(package='tf2_ros', executable='static_transform_publisher', name='static_transform_publisher_odom',
                output='screen', emulate_tty=True, arguments=['0.0', '0.0', '0.0', '0', '0', '0', 'world', 'base_link'],
                parameters=[{'use_sim_time': True}])


def gazebo_model_path():
    logger.debug('adding gazebo model env var')
    gazebo_model_path = [get_package_share_directory(package_name)]
    if 'GAZEBO_MODEL_PATH' in os.environ:
        gazebo_model_path.append(os.environ['GAZEBO_MODEL_PATH'])
    return SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH', ':'.join(gazebo_model_path),
        condition=IfCondition(start_gazebo)
    )
    

def gazebo_plugin_path():
    logger.debug('adding gazebo plugin path env var')
    gazebo_plugin_path = [os.path.join(get_package_prefix(package_name), 'lib')]
    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        gazebo_plugin_path.append(os.environ['GAZEBO_PLUGIN_PATH'])
    return SetEnvironmentVariable(
        'GAZEBO_PLUGIN_PATH', ':'.join(gazebo_plugin_path),
        condition=IfCondition(start_gazebo)
    )


def gazebo():
    logger.debug('adding_gazebo node')
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        launch_arguments={'verbose': 'false', 'pause': 'false',
                          'world': os.path.join(get_package_share_directory(package_name), world_file)}.items(),
        condition=IfCondition(start_gazebo))


def rviz():
    logger.debug('adding rivz2 node')
    return Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
                arguments=['-d', 'rviz/turtlebot3.rviz'],
                parameters=[{'use_sim_time': True}],
                condition=IfCondition(start_rviz))


def generate_launch_description():
    logger.debug('creating launch description')
    return IncludeLaunchDescription(
        [
            start_gazebo_arg(),
            start_rviz_arg(),
            teleop_type_arg(),
            world_file_arg(),
            gazebo(),
            rviz(),
            TimerAction(actions=[
                robot_state_publisher(),
                robot_spawn(),
                static_transform_publisher()
            ], period=5.0)
        ]
    )
