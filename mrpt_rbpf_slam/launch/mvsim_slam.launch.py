"""Launch file for RBPF SLAM with MVSim simulator."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for RBPF SLAM with MVSim simulator."""
    # Get package directories
    mrpt_rbpf_slam_share = FindPackageShare('mrpt_rbpf_slam')
    mvsim_share = FindPackageShare('mvsim')

    # Declare launch arguments
    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value=PathJoinSubstitution([
            mvsim_share, 'mvsim_tutorial', 'mvsim_slam.xml'
        ]),
        description='Path to the MVSim world file'
    )

    # MVSim simulator node
    mvsim_node = Node(
        package='mvsim',
        executable='mvsim_node',
        name='mvsim_simulator',
        output='screen',
        parameters=[{
            'world_file': LaunchConfiguration('world_file'),
            'do_fake_localization': False,  # Needed to run an external localization / SLAM system
        }]
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', PathJoinSubstitution([
            mvsim_share, 'mvsim_tutorial', 'mvsim_slam.rviz'
        ])]
    )

    # Set ROS console configuration
    set_rosconsole_config = SetEnvironmentVariable(
        name='ROSCONSOLE_CONFIG_FILE',
        value=PathJoinSubstitution([mrpt_rbpf_slam_share, 'config', 'rosconsole.config'])
    )

    # MRPT RBPF SLAM node
    slam_node = Node(
        package='mrpt_rbpf_slam',
        executable='mrpt_rbpf_slam',
        name='mrpt_rbpf_slam',
        output='screen',
        parameters=[
            {
                'ini_filename': PathJoinSubstitution([
                    mrpt_rbpf_slam_share, 'tutorial', 'grid_slam_demo.ini'
                ]),
                'odom_frame_id': 'odom',
                'global_frame_id': 'map',
                'base_frame_id': 'base_link',
                'sensor_source': 'laser1',
            },
            PathJoinSubstitution([mrpt_rbpf_slam_share, 'config', 'default.yaml'])
        ]
    )

    return LaunchDescription([
        world_file_arg,
        mvsim_node,
        rviz_node,
        set_rosconsole_config,
        slam_node,
    ])
