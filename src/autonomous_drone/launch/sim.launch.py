import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'autonomous_drone'
    file_subpath = 'description/quadcopter.urdf.xacro'


    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()


    # Configure the node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
        'use_sim_time': True}] # add other parameters here if required
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        )


    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_bot'],
        output='screen'
    )

    jmc_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["jmc"],
    )

    rf2o_laser_odometry = Node(
        package="rf2o_laser_odometry",
        executable="rf2o_laser_odometry_node",
        parameters=[{
                    'laser_scan_topic' : '/laser_controller/out',
                    'odom_topic' : '/odom',
                    'publish_tf' : True,
                    'base_frame_id' : 'base_link',
                    #'odom_frame_id' : 'odom',
                    'init_pose_from_topic' : '',
                    'freq' : 10.0}]

    )

    # Run the node
    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        jmc_spawner,
        rf2o_laser_odometry
    ])