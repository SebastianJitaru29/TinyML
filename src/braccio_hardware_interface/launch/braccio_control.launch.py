from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
  return LaunchDescription([
    DeclareLaunchArgument(
      'serial_port', default_value='/dev/ttyACM0',
      description='Serial port for Braccio'),

    Node(
      package='braccio_hardware_interface',
      executable='braccio_interface_node',
      name='braccio_interface_node',
      output='screen',
      parameters=[{
        'serial_port': LaunchConfiguration('serial_port'),
        'baud_rate': 115200,
        'joint_names': [
          'base_joint', 'shoulder_joint', 'elbow_joint',
          'wrist_pitch_joint', 'wrist_roll_joint'
        ],
        'publish_rate': 50.0
      }]
    ),
  ])
