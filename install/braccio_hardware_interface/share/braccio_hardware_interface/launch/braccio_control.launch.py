import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from rclpy.logging import get_logger
import traceback
from launch.actions import TimerAction

# Requires moveit_configs_utils installed
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    moveit_config_pkg_name = "braccio_moveit_config_fresh"
    robot_description_pkg_name = "braccio_description"

    urdf_xacro_filename_config = "braccio.urdf.xacro"
    srdf_filename = "braccio.srdf"
    controllers_filename = "ros2_controllers.yaml"
    moveit_controllers_filename = "moveit_controller.yaml"
    kinematics_filename = "kinematics.yaml"
    joint_limits_filename = "joint_limits.yaml"
    initial_positions_filename = "initial_positions.yaml"
    arm_controller_name = "braccio_arm_trajectory_controller"
    rviz_config_filename = "moveit.rviz"

    declared_arguments = [
        DeclareLaunchArgument("use_rviz", default_value="true", description="Launch RViz?"),
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyACM0", description="Serial port for Braccio"),
        DeclareLaunchArgument("baud_rate", default_value="115200", description="Baud rate for Braccio")
    ]

    use_rviz = LaunchConfiguration("use_rviz")
    serial_port_arg = LaunchConfiguration("serial_port")
    baud_rate_arg = LaunchConfiguration("baud_rate")

    robot_description_xacro_path = os.path.join(
        get_package_share_directory(moveit_config_pkg_name),
        "config",
        urdf_xacro_filename_config
    )

    initial_positions_file_path = os.path.join(
        get_package_share_directory(moveit_config_pkg_name),
        "config",
        initial_positions_filename
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory(moveit_config_pkg_name),
        "config",
        controllers_filename
    )

    rviz_config_path = os.path.join(
        get_package_share_directory(moveit_config_pkg_name),
        "config",
        rviz_config_filename
    )

    moveit_config = None
    try:
        get_logger("launch.user").info(f"--- Attempting MoveItConfigsBuilder setup for package: {moveit_config_pkg_name} ---")
        moveit_config = (
            MoveItConfigsBuilder("braccio", package_name=moveit_config_pkg_name)
            .robot_description(
                file_path=robot_description_xacro_path,
                mappings={"initial_positions_file": initial_positions_file_path}
            )
            .robot_description_semantic(file_path=f"config/{srdf_filename}")
            .robot_description_kinematics(file_path=f"config/{kinematics_filename}")
            .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
            .trajectory_execution(file_path=f"config/{moveit_controllers_filename}")
            .joint_limits(file_path=f"config/{joint_limits_filename}")
            .planning_scene_monitor(
                publish_planning_scene=True,
                publish_geometry_updates=True,
                publish_state_updates=True,
                publish_transforms_updates=True
            )
            .to_moveit_configs()
        )
        get_logger("launch.user").info("--- MoveItConfigsBuilder setup successful ---")

    except Exception as e:
        get_logger("launch.user").error(f"!!! Failed to build MoveItConfigs: {e} !!!")
        traceback.print_exc()
        return LaunchDescription([])

    if moveit_config is None or moveit_config.robot_description is None:
        get_logger("launch.user").fatal("!!! MoveItConfigs object is None or invalid. Aborting launch. !!!")
        return LaunchDescription([])

    robot_state_publisher_node = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        output="both", parameters=[moveit_config.robot_description],
    )

    ros2_control_node = Node(
        package="controller_manager", executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="screen",
    )

    braccio_hardware_node = Node(
        package="braccio_hardware_interface",
        executable="braccio_interface_node",
        output="screen",
        parameters=[{"serial_port": serial_port_arg, "baud_rate": baud_rate_arg}]
    )

    spawn_controllers = [
        TimerAction(
            period=5.0,  # Wait 5 seconds before spawning
            actions=[
                Node(
                    package="controller_manager", executable="spawner",
                    arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
                    output="screen",
                ),
            ]
        ),
        TimerAction(
            period=6.0,  # Wait an additional second to avoid overlap
            actions=[
                Node(
                    package="controller_manager", executable="spawner",
                    arguments=["braccio_arm_trajectory_controller", "-c", "/controller_manager"],
                    output="screen",
                ),
            ]
        ),
    ]


    move_group_node = Node(
        package="moveit_ros_move_group", executable="move_group",
        output="screen", parameters=[moveit_config.to_dict(), {"use_sim_time": False}]
    )

    rviz_node = Node(
        package="rviz2", executable="rviz2", name="rviz2", output="log",
        arguments=["-d", rviz_config_path],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(declared_arguments + [
        robot_state_publisher_node,
        ros2_control_node,
        braccio_hardware_node,
        move_group_node,
        rviz_node
    ] + spawn_controllers)
