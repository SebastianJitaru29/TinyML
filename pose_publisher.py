# pose_publisher.py
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/braccio_arm_trajectory_controller/joint_trajectory',
            10
        )
        self.timer = self.create_timer(2.0, self.publish_pose)
        self.get_logger().info("PosePublisher ready!")

    def publish_pose(self):
        msg = JointTrajectory()
        msg.joint_names = [
            "base_joint", "shoulder_joint", "elbow_joint",
            "wrist_pitch_joint", "wrist_roll_joint"
        ]

        point = JointTrajectoryPoint()
        point.positions = [2.0, 2.5, 2.2, 2.7, 2.3]  # Example pose
        point.time_from_start.sec = 2
        msg.points.append(point)

        self.publisher.publish(msg)
        self.get_logger().info(f"Published pose: {point.positions}")


def main(args=None):
    rclpy.init(args=args)
    node = PosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
