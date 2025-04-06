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

# Requires moveit_configs_utils installed
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    # --- Define Hardcoded Names (Adjust if necessary!) ---
    moveit_config_pkg_name = "braccio_moveit_config_fresh"
    robot_description_pkg_name = "braccio_description"
    # Correct TOP-LEVEL XACRO file that includes ros2_control info
    urdf_xacro_filename_config = "braccio.urdf.xacro"
    srdf_filename = "braccio.srdf"
    controllers_filename = "ros2_controllers.yaml" # Make sure this is correct
    moveit_controllers_filename = "moveit_controller.yaml" # Make sure this exists
    kinematics_filename = "kinematics.yaml"
    joint_limits_filename = "joint_limits.yaml"
    initial_positions_filename = "initial_positions.yaml" # Used by mock system
    arm_controller_name = "braccio_arm_trajectory_controller" # Must match controllers_filename
    rviz_config_filename = "moveit.rviz"

    # Declare only arguments that might change or need overriding
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument( "use_rviz", default_value="true", description="Launch RViz?")
    )
    declared_arguments.append(
        DeclareLaunchArgument( "serial_port", default_value="/dev/ttyACM0", description="Serial port for Braccio") # EDIT IF NEEDED
    )
    declared_arguments.append(
        DeclareLaunchArgument( "baud_rate", default_value="115200", description="Baud rate for Braccio")
    )

    # Initialize arguments needed for nodes
    use_rviz = LaunchConfiguration("use_rviz")
    serial_port_arg = LaunchConfiguration("serial_port")
    baud_rate_arg = LaunchConfiguration("baud_rate")

    # --- Construct File Paths ---
    # Path to the top-level URDF Xacro (in MoveIt config)
    robot_description_xacro_path = os.path.join(
        get_package_share_directory(moveit_config_pkg_name),
        "config",
        urdf_xacro_filename_config
    )
    # Path to the initial positions file needed by the xacro
    initial_positions_file_path = os.path.join(
        get_package_share_directory(moveit_config_pkg_name),
        "config",
        initial_positions_filename
    )
    # Path to the ROS 2 controllers config
    ros2_controllers_path = os.path.join(
        get_package_share_directory(moveit_config_pkg_name),
        "config",
        controllers_filename
    )
    # Path to the RViz config
    rviz_config_path = os.path.join(
        get_package_share_directory(moveit_config_pkg_name),
        "config",
        rviz_config_filename
    )

    # --- Use MoveItConfigsBuilder ---
    moveit_config = None
    try:
        get_logger("launch.user").info(f"--- Attempting MoveItConfigsBuilder setup for package: {moveit_config_pkg_name} ---")
        moveit_config = (
            MoveItConfigsBuilder("braccio", package_name=moveit_config_pkg_name)
            .robot_description(
                 file_path=robot_description_xacro_path, # Use path to correct top-level file
                 # Pass the initial_positions_file argument required by the mock setup xacro
                 mappings={"initial_positions_file": initial_positions_file_path}
             )
             # Use RELATIVE path STRINGS for files within the moveit_config package
            .robot_description_semantic(file_path=f"config/{srdf_filename}")
            .robot_description_kinematics(file_path=f"config/{kinematics_filename}")
            .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
            .trajectory_execution(file_path=f"config/{moveit_controllers_filename}") # MAKE SURE THIS FILE EXISTS
            .joint_limits(file_path=f"config/{joint_limits_filename}")
            .planning_scene_monitor(
                publish_planning_scene=True, publish_geometry_updates=True,
                publish_state_updates=True, publish_transforms_updates=True
             )
            .to_moveit_configs()
        )
        get_logger("launch.user").info("--- MoveItConfigsBuilder setup successful ---")

    except Exception as e:
        get_logger("launch.user").error(f"!!! Failed to build MoveItConfigs: {e} !!!")
        print("\n--- Detailed Traceback ---"); traceback.print_exc(); print("--- End Traceback ---\n")
        raise e

    # --- Check if moveit_config was successfully created ---
    if moveit_config is None or moveit_config.robot_description is None:
         get_logger("launch.user").fatal("!!! MoveItConfigs object is None or invalid. Aborting launch. !!!")
         return LaunchDescription([])

    # --- Nodes To Launch ---

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        output="both", parameters=[moveit_config.robot_description],
    )

    # Controller Manager (Loads mock hardware plugin based on URDF)
    ros2_control_node = Node(
        package="controller_manager", executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="screen", on_exit=[],
    )

    # Braccio Hardware Interface Node (Standalone Python script) - MUST BE LAUNCHED
    braccio_hardware_node = Node(
        package="braccio_hardware_interface",
        executable="braccio_interface_node", # Matches console_script entry point
        output="screen",
        # Pass parameters defined in the node
        parameters=[{"serial_port": serial_port_arg, "baud_rate": baud_rate_arg}],
    )

    # Spawners (Load controllers defined in ros2_controllers.yaml)
    spawn_controllers = [
        Node(
            package="controller_manager", executable="spawner",
            arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
            output="screen",
        ),
        Node(
            package="controller_manager", executable="spawner",
            arguments=[arm_controller_name, "-c", "/controller_manager"], # Use hardcoded name
            output="screen",
        ),
        # Gripper spawner removed
    ]

    # MoveGroup Node
    move_group_node = Node(
        package="moveit_ros_move_group", executable="move_group", output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": False}],
    )

    # RViz
    rviz_node = Node(
        package="rviz2", executable="rviz2", name="rviz2", output="log",
        arguments=["-d", rviz_config_path],
        parameters=[
            moveit_config.robot_description, moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines, moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        condition=IfCondition(use_rviz),
    )

    # --- Launch Description ---
    # ADD braccio_hardware_node back to this list
    nodes_to_start = [
        robot_state_publisher_node,
        ros2_control_node,
        braccio_hardware_node, # Launch the standalone hardware node
        move_group_node,
        rviz_node,
    ] + spawn_controllers # Add controller spawners

    return LaunchDescription(declared_arguments + nodes_to_start)