import serial
import time
import re 

BASE = 0
SHOULDER = 1
ELBOW = 2
WRIST_VERT = 3
WRIST_ROT = 4
GRIPPER = 5

HOME_ANGLES = [90, 90, 90, 90, 90, 73] # Base, Shoulder, Elbow, Wrist_V, Wrist_R, Gripper

class Mover():

    def __init__(self, port, baud_rate, timeout=5):
        self.serial = None
        self.port = port
        self.baudrate = baud_rate
        self.timeout = timeout
        self.current_angles = None
        print(f"Mover initialized for port {self.port} at {self.baudrate} baud.")

    def connect(self):
        """Establishes the serial connection and attempts to home the arm."""
        if self.serial and self.serial.is_open:
            print("Serial connection already open.")
            if self.current_angles is None: self.go_home() 
            return True
        try:
            print(f"Attempting to connect to {self.port}...")
            self.serial = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            print(f"Successfully connected to {self.port}.")
            print("Waiting a few seconds for Arduino to initialize...")
            time.sleep(3)

            print("Attempting to home the arm...")
            if self.go_home():
                 print("Arm homed successfully.")
                 return True
            else:
                 print("Warning: Failed to confirm home position.")
                 self.current_angles = list(HOME_ANGLES)
                 return True 

        except serial.SerialException as e:
            print(f"Error connecting to {self.port}: {e}")
            self.serial = None
            self.current_angles = None
            return False
        except Exception as e:
            print(f"An unexpected error occurred during connection: {e}")
            self.serial = None
            self.current_angles = None
            return False

    def send_command(self, command: str, update_state=True):
        """
        Sends a command string, prints response, and optionally updates
        internal state if it's a successful 'P' command.
        """
        if not self.serial or not self.serial.is_open:
            print("Error: Serial port not connected or open.")
            return None

        try:
            if not command.endswith('\n'):
                command += '\n'

            print(f"Sending command: {command.strip()}")
            self.serial.write(command.encode('utf-8'))
            raw_response = self.serial.readline()
            response_str = raw_response.decode('utf-8').strip()
            print(f'Received response: {response_str}')

            if update_state and command.startswith('P') and response_str == 'OK':
                match = re.match(r'P(\d+),(\d+),(\d+),(\d+),(\d+),(\d+),(\d+)', command.strip())
                if match:
                    new_angles = [int(a) for a in match.groups()[:6]] # Get first 6 groups (angles)
                    self.current_angles = new_angles
                else:
                     print("Warning: Could not parse sent P command to update state.")
            elif update_state and command.startswith('H') and response_str == 'OK':
                 self.current_angles = list(HOME_ANGLES)

            return response_str

        except serial.SerialException as e:
            print(f"Serial error during send/receive: {e}")
            return None
        except Exception as e:
            print(f"Error during send_command: {e}")
            return None

    def move_joint(self, joint_index: int, target_angle: int, speed: int = 50):
        if self.current_angles is None:
            print("Error: Current arm position unknown. Try connecting/homing first.")
            return False

        if not (0 <= joint_index <= 5):
            print(f"Error: Invalid joint index {joint_index}. Must be 0-5.")
            return False

        if not (0 <= target_angle <= 180):
             if joint_index == GRIPPER and (10 <= target_angle <= 73):
                  pass 
             else:
                  print(f"Warning: Target angle {target_angle} might be outside typical range (0-180).")

        if not (1 <= speed <= 180): 
             print(f"Warning: Speed {speed} might be outside typical range (1-180).")

        next_angles = list(self.current_angles)
        next_angles[joint_index] = target_angle


        angles_str = ','.join(map(str, next_angles))
        command = f"P{angles_str},{speed}\n"

        response = self.send_command(command)

        return response == 'OK' 

    def go_home(self, speed: int = 100):
        command = f"H\n"
        response = self.send_command(command, update_state=True)
        return response == 'OK'

    def disconnect(self):
        """Closes the serial connection."""
        if self.serial and self.serial.is_open:
            self.serial.close()
            print(f"Disconnected from {self.port}.")
            self.serial = None
            self.current_angles = None 
        else:
            print("Serial connection already closed or not established.")

    def __enter__(self):
        if self.connect():
            return self
        else:
             raise IOError(f"Failed to connect to serial port {self.port}")

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()