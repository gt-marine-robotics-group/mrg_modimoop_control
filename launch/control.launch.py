from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    ekf_yaml = PathJoinSubstitution([
        FindPackageShare("mrg_modimoop_control"),
        "config",
        "ekf.yaml",
    ])

    lqr_control_yaml = PathJoinSubstitution([
        FindPackageShare("mrg_modimoop_control"),
        "config",
        "lqr_heading_controller.yaml",
    ])

    default_schedule_csv_path = PathJoinSubstitution([
        FindPackageShare("mrg_modimoop_control"),
        "params",
        "control_schedule.csv",
    ])

    schedule_csv_arg = DeclareLaunchArgument(
        "schedule_csv_path",
        default_value=default_schedule_csv_path,
        description="Path to offline LQR control schedule CSV",
    )

    return LaunchDescription([

        schedule_csv_arg,

        Node(
            package="mrg_modimoop_control",
            executable="lqr_heading_controller",
            name="lqr_heading_controller",
            output="screen",
            parameters=[lqr_control_yaml,
                        {"schedule_csv_path": LaunchConfiguration("schedule_csv_path")}],
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