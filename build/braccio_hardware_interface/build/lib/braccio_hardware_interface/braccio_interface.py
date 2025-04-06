#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial, time, math, threading
from trajectory_msgs.msg import JointTrajectory

# --- Constants ---
JOINT_NAMES = ["base_joint", "shoulder_joint", "elbow_joint", "wrist_pitch_joint", "wrist_roll_joint"]
NUM_JOINTS = len(JOINT_NAMES)
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200
RAD_TO_DEG = 180.0 / math.pi
DEG_CLAMP = [(0, 180)] * 5  # Limits for each joint in degrees
SEND_SPEED = 80

class BraccioHardwareInterface(Node):
    def __init__(self):
        super().__init__('braccio_hardware_interface')
        self.get_logger().info("Starting Braccio Hardware Interface Node...")

        self._lock = threading.Lock()
        self._serial = None
        self._last_command = [90] * NUM_JOINTS  # default home pos

        self.connect_serial()

        # Subscribe to trajectory topic used by MoveIt
        self.create_subscription(
            JointTrajectory,
            "/braccio_arm_trajectory_controller/joint_trajectory",
            self.trajectory_callback,
            10
        )

    def connect_serial(self):
        try:
            with self._lock:
                self._serial = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1.0, write_timeout=1.0)
            self.get_logger().info("Serial connected. Waiting for Arduino reset...")
            time.sleep(3.0)
            self.send_serial("H\n", wait_for_ok=True)
        except Exception as e:
            self.get_logger().error(f"Failed to connect to serial: {e}")

    def trajectory_callback(self, msg: JointTrajectory):
        if len(msg.points) == 0:
            self.get_logger().warn("Received empty trajectory")
            return

        latest_point = msg.points[-1]  # Take final pose in trajectory
        if len(latest_point.positions) != NUM_JOINTS:
            self.get_logger().error(f"Expected {NUM_JOINTS} joints, got {len(latest_point.positions)}")
            return

        with self._lock:
            target_deg = [
                max(min(int(round(pos * RAD_TO_DEG)), DEG_CLAMP[i][1]), DEG_CLAMP[i][0])
                for i, pos in enumerate(latest_point.positions)
            ]

        cmd_str = f"P{','.join(map(str, target_deg))},{SEND_SPEED}\n"
        self.get_logger().info(f"Sending command: {cmd_str.strip()}")
        if self.send_serial(cmd_str):
            self._last_command = target_deg

    def send_serial(self, cmd, wait_for_ok=False, timeout=2.0):
        try:
            with self._lock:
                self._serial.reset_input_buffer()
                self._serial.write(cmd.encode('utf-8'))
                self._serial.flush()

                if wait_for_ok:
                    start = time.monotonic()
                    while time.monotonic() - start < timeout:
                        if self._serial.in_waiting > 0:
                            resp = self._serial.readline().decode().strip()
                            if resp == "OK":
                                return True
                        time.sleep(0.01)
                    self.get_logger().warn("Timeout waiting for OK from Arduino")
                    return False
                return True
        except Exception as e:
            self.get_logger().error(f"Serial error: {e}")
            return False

    def destroy_node(self):
        if self._serial and self._serial.is_open:
            self._serial.close()
            self.get_logger().info("Serial connection closed.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BraccioHardwareInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down via keyboard interrupt.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("ROS 2 shutdown complete.")


if __name__ == '__main__':
    main()
