import os
import random
import datetime
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_prefix

import xacro


def generate_launch_description():

    package_name = "turtlebot3_description"
    install_dir = get_package_prefix(package_name)

    gazebo_models_path = os.path.join(package_name, 'urdf')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] = os.environ['GAZEBO_MODEL_PATH'] + \
            ':' + install_dir + '/share' + ':' + gazebo_models_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = install_dir + \
            "/share" + ':' + gazebo_models_path

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + \
            ':' + install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'

    print("GAZEBO MODELS PATH=="+str(os.environ["GAZEBO_MODEL_PATH"]))
    print("GAZEBO PLUGINS PATH=="+str(os.environ["GAZEBO_PLUGIN_PATH"]))

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        launch_arguments={"verbose": "false", 'pause': 'true'}.items(),
    )

    robot_model_path = os.path.join(
        get_package_share_directory(package_name))

    xacro_file = os.path.join(robot_model_path, 'urdf', 'turtlebot3_burger.urdf.xacro')

    # convert XACRO file into URDF
    print("parsing xacro doc")
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    # save for debugging
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    outfile_file = os.path.join("/tmp", f"{os.path.splitext(os.path.split(xacro_file)[-1])[0]}-{timestamp}.urdf")
    with open(outfile_file, 'w') as f:
        f.write(doc.toprettyxml(indent="    "))

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Position and orientation
    # [X, Y, Z]
    position = [0.0, 0.0, 0.2]
    # [Roll, Pitch, Yaw]
    orientation = [0.0, 0.0, 0.0]
    # Base Name or robot
    robot_base_name = "turtlebot3"


    entity_name = robot_base_name+"-"+str(int(random.random()*100000))

    # Spawn ROBOT Set Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-entity',
                   entity_name,
                   '-x', str(position[0]), '-y', str(position[1]
                                                     ), '-z', str(position[2]),
                   '-R', str(orientation[0]), '-P', str(orientation[1]
                                                        ), '-Y', str(orientation[2]),
                   '-topic', '/robot_description'
                   ]
    )

        # RVIZ Configuration
    rviz_config_file = os.path.join(get_package_share_directory(package_name), 'rviz', 'burger_ros2.rviz')
    print(f'rviz config dir: {rviz_config_file}')

    print('creating rviz2 node')
    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config_file])

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        rviz_node,
    ])
