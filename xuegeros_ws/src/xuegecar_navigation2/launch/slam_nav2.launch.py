"""SLAM + Nav2 for Jazzy (standalone nodes, no nav2_bringup container)."""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    nav2_dir = get_package_share_directory('xuegecar_navigation2')
    slam_gmapping_dir = get_package_share_directory('slam_gmapping')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_yaml = LaunchConfiguration('map', default=os.path.join(nav2_dir, 'maps', 'room.yaml'))

    costmap_common = {
        'global_frame': 'odom',
        'robot_base_frame': 'base_link',
        'rolling_window': False,
        'width': 5, 'height': 5, 'resolution': 0.05,
        'robot_radius': 0.1,
        'always_send_full_costmap': True,
        'plugins': ['static_layer', 'inflation_layer'],
        'inflation_layer': {
            'plugin': 'nav2_costmap_2d::InflationLayer',
            'cost_scaling_factor': 3.0, 'inflation_radius': 0.2,
        },
    }

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map', default_value=map_yaml),

        # --- SLAM ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_gmapping_dir, 'launch', 'slam_gmapping.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        # --- map_server ---
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             parameters=[{'use_sim_time': use_sim_time, 'yaml_filename': map_yaml}],
             output='screen'),

        # --- AMCL ---
        Node(package='nav2_amcl', executable='amcl', name='amcl',
             parameters=[{'use_sim_time': use_sim_time,
                          'base_frame_id': 'base_link', 'global_frame_id': 'map',
                          'odom_frame_id': 'odom', 'scan_topic': 'scan',
                          'robot_model_type': 'nav2_amcl::DifferentialMotionModel',
                          'alpha1': 0.2, 'alpha2': 0.2, 'alpha3': 0.2, 'alpha4': 0.2, 'alpha5': 0.2,
                          'min_particles': 500, 'max_particles': 2000,
                          'update_min_d': 0.25, 'update_min_a': 0.2,
                          'transform_tolerance': 1.0, 'tf_broadcast': True, 'set_initial_pose': True}],
             output='screen'),

        # --- controller_server ---
        Node(package='nav2_controller', executable='controller_server', name='controller_server',
             parameters=[{'use_sim_time': use_sim_time, 'controller_frequency': 20.0,
                          'controller_plugins': ['FollowPath'],
                          'progress_checker_plugin': 'progress_checker',
                          'goal_checker_plugins': ['general_goal_checker'],
                          'progress_checker': {'plugin': 'nav2_controller::SimpleProgressChecker',
                                                'required_movement_radius': 0.5, 'movement_time_allowance': 10.0},
                          'general_goal_checker': {'plugin': 'nav2_controller::SimpleGoalChecker',
                                                    'xy_goal_tolerance': 0.25, 'yaw_goal_tolerance': 0.25, 'stateful': True},
                          'FollowPath': {'plugin': 'dwb_core::DWBLocalPlanner',
                                          'min_vel_x': -0.12, 'max_vel_x': 0.3, 'max_vel_theta': 1.2,
                                          'vx_samples': 20, 'vtheta_samples': 20, 'transform_tolerance': 0.5,
                                          'critics': ['RotateToGoal', 'BaseObstacle', 'PathAlign', 'PathDist', 'GoalAlign', 'GoalDist'],
                                          'PathAlign.scale': 32.0, 'GoalAlign.scale': 24.0,
                                          'PathDist.scale': 32.0, 'GoalDist.scale': 24.0, 'RotateToGoal.scale': 32.0},
                          'qos_overrides': {'/cmd_vel': {'publisher': {'reliability': 'best_effort', 'durability': 'volatile'}}},
                          **costmap_common}],
             output='screen'),

        # --- planner_server ---
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             parameters=[{'use_sim_time': use_sim_time, 'planner_plugins': ['GridBased'],
                          'expected_planner_frequency': 20.0,
                          'GridBased': {'plugin': 'nav2_navfn_planner::NavfnPlanner',
                                         'tolerance': 0.5, 'use_astar': False, 'allow_unknown': True}}],
             output='screen'),

        # --- bt_navigator ---
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
             parameters=[{'use_sim_time': use_sim_time, 'global_frame': 'map',
                          'robot_base_frame': 'base_link', 'odom_topic': '/odom', 'default_server_timeout': 20}],
             output='screen'),

        # --- behavior_server ---
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server',
             parameters=[{'use_sim_time': use_sim_time,
                          'costmap_topic': 'local_costmap/costmap_raw',
                          'footprint_topic': 'local_costmap/published_footprint',
                          'cycle_frequency': 10.0,
                          'behavior_plugins': ['spin', 'backup', 'wait'],
                          'spin': {'plugin': 'nav2_behaviors::Spin'},
                          'backup': {'plugin': 'nav2_behaviors::BackUp'},
                          'wait': {'plugin': 'nav2_behaviors::Wait'},
                          'global_frame': 'odom', 'robot_base_frame': 'base_link',
                          'transform_tolerance': 0.5}],
             output='screen'),

        # --- lifecycle_manager ---
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation',
             parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                          'node_names': ['map_server', 'amcl', 'controller_server',
                                          'planner_server', 'bt_navigator', 'behavior_server']}],
             output='screen'),
    ])
