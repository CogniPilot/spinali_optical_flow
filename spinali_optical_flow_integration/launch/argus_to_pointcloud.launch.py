from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Package paths
    pkg_integration = Path(get_package_share_directory('spinali_optical_flow_integration'))
    pkg_description = Path(get_package_share_directory('spinali_optical_flow_description'))

    # Xacro file path
    xacro_file = pkg_description / 'urdf' / 'spinali_optical_flow.urdf.xacro'

    # RViz config path
    rviz_config = pkg_integration / 'rviz' / 'argus_pointcloud.rviz'

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            choices=['true', 'false'],
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'use_full_visual',
            default_value='false',
            choices=['true', 'false'],
            description='Use full detailed meshes instead of minimal geometry'
        ),
        DeclareLaunchArgument(
            'no_case',
            default_value='true',
            choices=['true', 'false'],
            description='Exclude case (base and spacer) from visualization'
        ),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            choices=['true', 'false'],
            description='Launch RViz for visualization'
        ),
        # Synapse ROS arguments
        DeclareLaunchArgument(
            'host',
            default_value='192.0.2.1',
            description='Host for cerebri'
        ),
        DeclareLaunchArgument(
            'port',
            default_value='4242',
            description='TCP port for cerebri'
        ),
        DeclareLaunchArgument(
            'rpmsg_dev',
            default_value='',
            description='RPMsg character device (used instead of sockets when configured)'
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='error',
            choices=['info', 'warn', 'error'],
            description='Log level'
        ),
        # Argus to PointCloud arguments
        DeclareLaunchArgument(
            'argus_topic',
            default_value='/spinali/out/argus_results',
            description='Argus results topic'
        ),
        DeclareLaunchArgument(
            'output_topic',
            default_value='argus_pointcloud',
            description='Output topic for PointCloud2 messages'
        ),
        DeclareLaunchArgument(
            'pixel_pitch_x',
            default_value='0.0005',
            description='Pixel pitch in X direction (meters)'
        ),
        DeclareLaunchArgument(
            'pixel_pitch_y',
            default_value='0.0005',
            description='Pixel pitch in Y direction (meters)'
        ),
        DeclareLaunchArgument(
            'fov_horizontal',
            default_value='2.0',
            description='Horizontal field of view per pixel (degrees)'
        ),
        DeclareLaunchArgument(
            'fov_vertical',
            default_value='2.0',
            description='Vertical field of view per pixel (degrees)'
        ),
        DeclareLaunchArgument(
            'parent_frame',
            default_value='world',
            description='Parent TF frame'
        ),
        DeclareLaunchArgument(
            'sensor_frame',
            default_value='afbr-s50lx85d',
            description='Sensor TF frame for pointcloud output'
        ),
        DeclareLaunchArgument(
            'tf_child_frame',
            default_value='spinali_optical_flow_link',
            description='Child TF frame for IMU orientation (published by this node)'
        ),
        DeclareLaunchArgument(
            'publish_tf',
            default_value='true',
            description='Whether to publish TF transform for IMU orientation'
        ),
        DeclareLaunchArgument(
            'tf_x',
            default_value='0.0',
            description='Transform X offset (meters)'
        ),
        DeclareLaunchArgument(
            'tf_y',
            default_value='0.0',
            description='Transform Y offset (meters)'
        ),
        DeclareLaunchArgument(
            'tf_z',
            default_value='0.0',
            description='Transform Z offset (meters)'
        ),
        DeclareLaunchArgument(
            'tf_roll',
            default_value='0.0',
            description='Transform roll (radians)'
        ),
        DeclareLaunchArgument(
            'tf_pitch',
            default_value='0.0',
            description='Transform pitch (radians)'
        ),
        DeclareLaunchArgument(
            'tf_yaw',
            default_value='0.0',
            description='Transform yaw (radians)'
        ),
        DeclareLaunchArgument(
            'use_imu_orientation',
            default_value='true',
            description='Use IMU/magnetometer fusion for orientation'
        ),
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/spinali/out/imu',
            description='IMU topic (when use_imu_orientation is true)'
        ),
        DeclareLaunchArgument(
            'mag_topic',
            default_value='/spinali/out/magnetic_field',
            description='Magnetometer topic (when use_imu_orientation is true)'
        ),
        DeclareLaunchArgument(
            'madgwick_beta',
            default_value='0.1',
            description='Madgwick filter gain'
        ),

        # Synapse ROS node (with spinali namespace)
        Node(
            namespace='spinali',
            package='synapse_ros',
            executable='synapse_ros',
            name='synapse_ros',
            output='screen',
            parameters=[{
                'host': LaunchConfiguration('host'),
                'port': LaunchConfiguration('port'),
                'rpmsg_dev': LaunchConfiguration('rpmsg_dev'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            remappings=[
                ('/spinali/in/cmd_vel', '/cmd_vel'),
            ],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            on_exit=Shutdown(),
        ),

        # Robot State Publisher (publishes URDF and TF from description)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'robot_description': Command([
                    'xacro ', str(xacro_file),
                    ' use_full_visual:=', LaunchConfiguration('use_full_visual'),
                    ' no_case:=', LaunchConfiguration('no_case')
                ])},
            ],
        ),

        # Argus to PointCloud node
        Node(
            package='spinali_optical_flow_integration',
            executable='argus_to_pointcloud',
            name='argus_to_pointcloud',
            output='screen',
            parameters=[{
                'argus_topic': LaunchConfiguration('argus_topic'),
                'output_topic': LaunchConfiguration('output_topic'),
                'pixel_pitch_x': LaunchConfiguration('pixel_pitch_x'),
                'pixel_pitch_y': LaunchConfiguration('pixel_pitch_y'),
                'fov_horizontal': LaunchConfiguration('fov_horizontal'),
                'fov_vertical': LaunchConfiguration('fov_vertical'),
                'parent_frame': LaunchConfiguration('parent_frame'),
                'sensor_frame': LaunchConfiguration('sensor_frame'),
                'tf_child_frame': LaunchConfiguration('tf_child_frame'),
                'publish_tf': LaunchConfiguration('publish_tf'),
                'tf_x': LaunchConfiguration('tf_x'),
                'tf_y': LaunchConfiguration('tf_y'),
                'tf_z': LaunchConfiguration('tf_z'),
                'tf_roll': LaunchConfiguration('tf_roll'),
                'tf_pitch': LaunchConfiguration('tf_pitch'),
                'tf_yaw': LaunchConfiguration('tf_yaw'),
                'use_imu_orientation': LaunchConfiguration('use_imu_orientation'),
                'imu_topic': LaunchConfiguration('imu_topic'),
                'mag_topic': LaunchConfiguration('mag_topic'),
                'madgwick_beta': LaunchConfiguration('madgwick_beta'),
            }],
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', str(rviz_config)],
            condition=IfCondition(LaunchConfiguration('launch_rviz')),
        ),
    ])
