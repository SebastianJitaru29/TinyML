#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial, time, math, threading
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState

# --- Constants ---
JOINT_NAMES = ["base_joint", "shoulder_joint", "elbow_joint", "wrist_pitch_joint", "wrist_roll_joint"]
NUM_JOINTS = len(JOINT_NAMES)
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200
RAD_TO_DEG = 180.0 / math.pi
DEG_CLAMP = [(0, 180)] * NUM_JOINTS  # Degree limits
SEND_SPEED = 80

class BraccioHardwareInterface(Node):
    def __init__(self):
        super().__init__('braccio_hardware_interface')
        self.get_logger().info("Starting Braccio Hardware Interface Node...")

        self._lock = threading.Lock()
        self._serial = None
        self._last_command = [90] * NUM_JOINTS  # default home

        self.connect_serial()

        self.create_subscription(
            JointTrajectory,
            "/braccio_arm_trajectory_controller/joint_trajectory",
            self.trajectory_callback,
            10
        )

        self.joint_state_pub = self.create_publisher(JointState, "/joint_state", 10)
        self._current_state_rad = [math.radians(a) for a in self._last_command]
        self.create_timer(0.05, self.publish_joint_state)  # 20 Hz


    def publish_joint_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = self._current_state_rad
        msg.velocity = [0.0] * NUM_JOINTS
        msg.effort = [0.0] * NUM_JOINTS
        self.joint_state_pub.publish(msg)


    def connect_serial(self):
        try:
            with self._lock:
                self._serial = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1.0)
            self.get_logger().info(f"Connected to {SERIAL_PORT} at {BAUD_RATE}")
            time.sleep(3.0)
            self.send_serial("H\n")
        except Exception as e:
            self.get_logger().error(f"Serial connection failed: {e}")

    def trajectory_callback(self, msg: JointTrajectory):
        if not msg.points:
            self.get_logger().warn("Received empty trajectory")
            return

        latest = msg.points[-1]
        if len(latest.positions) != NUM_JOINTS:
            self.get_logger().error(f"Expected {NUM_JOINTS} joints, got {len(latest.positions)}")
            return

        with self._lock:
            target_deg = [
                max(min(int(round(pos * RAD_TO_DEG)), DEG_CLAMP[i][1]), DEG_CLAMP[i][0])
                for i, pos in enumerate(latest.positions)
            ]

            cmd = f"P{','.join(map(str, target_deg))},{SEND_SPEED}\n"
            self.get_logger().info(f"Sending command: {cmd.strip()}")
            self.send_serial(cmd)
            self._current_state_rad = [math.radians(a) for a in target_deg]


    def send_serial(self, command, timeout=2.0):
        try:
            self._serial.write(command.encode('utf-8'))
            response = self._serial.readline().decode('utf-8').strip()
            self.get_logger().info(f"Arduino response: {response}")
            return response == "OK"
        except Exception as e:
            self.get_logger().error(f"Serial send failed: {e}")
            return False

    def destroy_node(self):
        if self._serial and self._serial.is_open:
            self._serial.close()
            self.get_logger().info("Serial port closed.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = BraccioHardwareInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown requested via keyboard.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("ROS 2 shutdown complete.")

if __name__ == '__main__':
    main()
