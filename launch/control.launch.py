from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    sail_table_csv = PathJoinSubstitution([
        FindPackageShare("mrg_modimoop_control"),
        "params",
        "vlm_tables.csv",
    ])

    ekf_yaml = PathJoinSubstitution([
        FindPackageShare("mrg_modimoop_control"),
        "config",
        "ekf.yaml",
    ])

    control_yaml = PathJoinSubstitution([
        FindPackageShare("mrg_modimoop_control"),
        "config",
        "control.yaml",
    ])

    return LaunchDescription([
        Node(
            package="mrg_modimoop_control",
            executable="control_bridge",
            name="control_bridge",
            parameters=[control_yaml,
                        {
                            "sail_table_csv": sail_table_csv,
                        }
                    ],
            output="screen",
        ),

        Node(
            package="mrg_modimoop_control",
            executable="localization",
            name="localization",
            output="screen",
            parameters=[{
                'declination_rad': 0.0,
                'use_tilt_compensation': True,
                'publish_rate_hz': 20.0,
            }]
        ),

        Node(
            package="mrg_modimoop_control",
            executable="planner",
            name="planner",
            output="screen",
            parameters=[{
                'g_goal': 10.0,
                'g_up': 5.0,
                'g_down': 5.0,
                'g_hyst': 2.0,
                'phi_up_rad': 0.52,
                'window_size_m': 5.0,
                'sample_step_deg': 5.0,
            }]
        ),

        #Node(
        #    package="mrg_modimoop_control",
        #    executable="temp_goal_publisher",
        #    name="temp_goal_publisher",
        #    output="screen",
        #    parameters=[{
        #        'publish_rate_hz': 2.0,
        #        'east_offset_m': -1000.0,
        #        'north_offset_m': 0.0,
        #    }]
        #),

        # We have to fuse IMU and Magnetometer data into one; this is a bit of a workaround
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            output='screen',
            parameters=[{
                'use_magnetic_field_msg': True,
                'world_frame': 'enu',
                'publish_tf': False,
                'use_sim_time': True,
            }],
            remappings=[
                ('/imu/data_raw', '/modimoop/imu_raw'),
                ('/imu/mag', '/modimoop/mag'),
                ('/imu/data', '/modimoop/imu'),
            ]
        ),

        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_local",
            parameters=[ekf_yaml],
            remappings=[
                ("/odometry/filtered", "/modimoop/localization/odometry/local"),
            ],
        ),

        Node(
            package="robot_localization",
            executable="navsat_transform_node",
            name="navsat_transform",
            parameters=[ekf_yaml],
            remappings=[
                ("/imu/data", "/modimoop/imu"),
                ("/gps/fix", "/modimoop/navsat"),
                ("/odometry/filtered", "/modimoop/localization/odometry/global"),
                ("/odometry/gps", "/modimoop/localization/odometry/gps"),
                ("/gps/filtered", "/modimoop/localization/gps/filtered"),
            ],
        ),

        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_global",
            parameters=[ekf_yaml],
            remappings=[
                ("/odometry/filtered", "/modimoop/localization/odometry/global"),
            ],
        ),

    ])