"""Nav2 minimal launch for Jazzy — RegulatedPurePursuit + scan QoS relay."""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_yaml = LaunchConfiguration('map')

    return LaunchDescription([
        DeclareLaunchArgument('map'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        # --- QoS relay: micro-ROS /scan (BEST_EFFORT) -> /scan_relay (RELIABLE) ---
        Node(
            package='xuegecar_navigation2', executable='scan_relay.py',
            name='scan_relay', output='screen',
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
                          'qos_overrides': {
                              '/scan': {'subscription': {'reliability': 'best_effort', 'durability': 'volatile'}},
                          },
                          'robot_model_type': 'nav2_amcl::DifferentialMotionModel',
                          'alpha1': 0.2, 'alpha2': 0.2, 'alpha3': 0.2, 'alpha4': 0.2, 'alpha5': 0.2,
                          'min_particles': 500, 'max_particles': 2000,
                          'update_min_d': 0.25, 'update_min_a': 0.2,
                          'transform_tolerance': 3.0, 'tf_broadcast': True, 'set_initial_pose': True}],
             output='screen'),

        # --- controller_server (RegulatedPurePursuit) ---
        Node(package='nav2_controller', executable='controller_server', name='controller_server',
             parameters=[{
                 'use_sim_time': use_sim_time,
                 'controller_frequency': 20.0,
                 'controller_plugins': ['FollowPath'],
                 'FollowPath': {
                     'plugin': 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
                     'desired_linear_vel': 0.2,
                     'lookahead_dist': 0.4,
                     'min_lookahead_dist': 0.3,
                     'max_lookahead_dist': 0.9,
                     'transform_tolerance': 0.5,
                 },
                 'qos_overrides': {
                     '/cmd_vel': {'publisher': {'reliability': 'best_effort', 'durability': 'volatile'}},
                 },
             }],
             output='screen'),

        # --- planner_server ---
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             parameters=[{'use_sim_time': use_sim_time,
                          'planner_plugins': ['GridBased'],
                          'expected_planner_frequency': 20.0,
                          'GridBased': {'plugin': 'nav2_navfn_planner::NavfnPlanner',
                                        'tolerance': 0.5, 'use_astar': False, 'allow_unknown': True}}],
             output='screen'),

        # --- bt_navigator ---
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
             parameters=[{'use_sim_time': use_sim_time, 'global_frame': 'map',
                          'robot_base_frame': 'base_link', 'odom_topic': '/odom',
                          'default_server_timeout': 20}],
             output='screen'),

        # --- behavior_server ---
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server',
             parameters=[{'use_sim_time': use_sim_time,
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
