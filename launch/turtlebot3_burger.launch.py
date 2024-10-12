import logging
import os
from os import PathLike
import datetime
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_prefix

import xacro

log_level = os.environ.get('LOG_LEVEL', 'INFO').upper()
logging.basicConfig(level=log_level)
logging.info('logging configured')

robot_name = 'turtlebot3'
package_name = 'turtlebot3_description'
base_xacro_file = 'turtlebot3_burger.urdf.xacro'
install_dir = get_package_prefix(package_name)
robot_model_path = os.path.join(get_package_share_directory(package_name))
xacro_file = os.path.join(robot_model_path, 'urdf', base_xacro_file)
x = '0.0'
y = '0.0'
z = '0.0'


def process_xacro_file(xacro_file: PathLike) -> str:
    # convert XACRO file into URDF
    logging.debug('parsing xacro doc')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    return doc.toprettyxml(indent='    ')


def set_gazebo_env_vars():
    gazebo_models_path = os.path.join(package_name, 'urdf')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] = os.environ['GAZEBO_MODEL_PATH'] + \
                                          ':' + install_dir + '/share' + ':' + gazebo_models_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = install_dir + \
                                          '/share' + ':' + gazebo_models_path

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + \
                                           ':' + install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'

    logging.debug('GAZEBO MODELS PATH==' + str(os.environ['GAZEBO_MODEL_PATH']))
    logging.debug('GAZEBO PLUGINS PATH==' + str(os.environ['GAZEBO_PLUGIN_PATH']))


def generate_robot_state_publisher_node():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    urdf_text = process_xacro_file(xacro_file)
    # save it for debugging
    timestamp = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    outfile_file = os.path.join('/tmp', f'{os.path.splitext(os.path.split(xacro_file)[-1])[0]}-{timestamp}.urdf')
    with open(outfile_file, 'w') as f:
        f.write(urdf_text)

    return Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': urdf_text}],
        output='screen'
    )


def generate_robot_spawn():
    logging.debug('robot spawn topic is robot_description')
    return Node(
        name='spawn_entity',
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', robot_name, '-x', x, '-y', y, '-z', z, '-topic', 'robot_description']
    )


def genrate_static_transform_publisher_node(world_frame):
    logging.debug('creating static transform')
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_odom',
        output='screen',
        emulate_tty=True,
        arguments=[x, y, z, '0', '0', '0', world_frame, 'base_link']
    )


def generate_launch_description():
    logging.debug('starting generate_launch_description')
    set_gazebo_env_vars()
    world_frame = 'world'
    world_path = 'worlds/tenBySixteenRoom.world'
    full_world_path = os.path.join(get_package_share_directory(package_name), world_path)
    logging.info(f'world file: {full_world_path}')
    logging.debug('creating launch description')
    launch_description_items = [
        DeclareLaunchArgument(
            'world',
            default_value=full_world_path,
            description='Path to the Gazebo world file'
        ),
        LogInfo(msg='Launching gazebo.'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
            launch_arguments={
                'verbose': 'false',
                'pause': 'false',
                'world': LaunchConfiguration('world'),
            }.items(),
        ),
    ]
    logging.debug('created base launch config, now appending stuff')
    delayed_start = TimerAction(
        actions=[],
        period=5.0
    )
    launch_description_items.append(LogInfo(msg='Gazebo started. Launching robot nodes.'))
    launch_description_items.append(delayed_start)
    logging.debug('appended delay start')
    # start the robots after waiting a bit for gazebo
    logging.debug(f'defining {robot_name}')
    state_publisher = generate_robot_state_publisher_node()
    logging.debug('state publisher added')
    delayed_start.actions.append(state_publisher)
    logging.debug('delayed start added')
    robot_spawn = generate_robot_spawn()
    logging.debug('robot spawn created')
    delayed_start.actions.append(robot_spawn)
    logging.debug('robot spawn added')
    static_transform = genrate_static_transform_publisher_node(world_frame)
    logging.debug('static transform created')
    delayed_start.actions.append(static_transform)
    logging.debug('static transform added')
    return LaunchDescription(launch_description_items)
