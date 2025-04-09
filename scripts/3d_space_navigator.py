import numpy as np
import math
import tkinter as tk
from tkinter import ttk
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import serial
import time

# ---------------------------------------------------------
# 1. Define the Braccio-like 4-DOF Kinematic Chain with ikpy
# ---------------------------------------------------------
# We include only the following actuated joints:
#   M1: Base rotation
#   M2: Shoulder
#   M3: Elbow
#   M4: Vertical wrist
#
# Dimensions are given in meters.
L1 = 0.05   # Vertical offset from base to joint1 (M1)
L2 = 0.10   # Length from joint1 to joint2 (M2: Shoulder)
L3 = 0.10   # Length from joint2 to joint3 (M3: Elbow)
L4 = 0.05   # Length from joint3 to joint4 (M4: Vertical wrist)

# Create the chain with 5 links:
# Index 0: Fixed OriginLink (base), then joints for M1 to M4.
braccio_chain = Chain(name='braccio', links=[
    OriginLink(),  # Fixed base (index 0); not actuated.
    URDFLink(
        name="joint1", 
        origin_translation=[0, 0, L1],
        origin_orientation=[0, 0, 0],
        rotation=[0, 0, 1],  # Rotation about z-axis for base.
        bounds=(np.deg2rad(0), np.deg2rad(180))
    ),
    URDFLink(
        name="joint2",
        origin_translation=[L2, 0, 0],
        origin_orientation=[0, 0, 0],
        rotation=[0, 1, 0],  # Shoulder rotates about y-axis.
        bounds=(np.deg2rad(15), np.deg2rad(165))
    ),
    URDFLink(
        name="joint3",
        origin_translation=[L3, 0, 0],
        origin_orientation=[0, 0, 0],
        rotation=[0, 1, 0],  # Elbow rotates about y-axis.
        bounds=(np.deg2rad(0), np.deg2rad(180))
    ),
    URDFLink(
        name="joint4",
        origin_translation=[L4, 0, 0],
        origin_orientation=[0, 0, 0],
        rotation=[0, 0, 1],  # Vertical wrist rotates about z-axis.
        bounds=(np.deg2rad(0), np.deg2rad(180))
    )
])

# ----------------------------------------------
# 2. Inverse Kinematics Computation Function
# ----------------------------------------------
def compute_braccio_angles(x, y, z, initial_position=None):
    """
    Computes the inverse kinematics solution for the simplified Braccio-like robot.
    Only the actuated joints for M1–M4 are included.
    
    In this formulation:
      - The target position (x, y, z) is provided in the robot's coordinate frame.
      - The z coordinate is adjusted by subtracting L1, because the fixed base (OriginLink)
        lifts the shoulder (M1) above the robot’s true base.
      - The target orientation is set to a rotation of -90° about the x-axis,
        which typically aligns the end‑effector to extend forward.
    
    Parameters:
      x, y, z : float
          Target coordinates in meters.
      initial_position : list or None
          Optional initial guess for all links (including the fixed OriginLink). This
          must be a list of 5 values (one per link). If any value is > 2π, they are assumed
          to be in degrees and will be converted to radians. If not provided, a vector of zeros
          is used.
    
    Returns:
      joint_angles_deg : list of float
          The computed joint angles for motors M1, M2, M3, and M4 (in degrees).
          Returns None if no solution is found.
    """
    # Adjust the z coordinate to account for the fixed base offset.
    adjusted_z = z - L1
    
    # Define the target position as a 3-element vector.
    target_position = np.array([x, y, adjusted_z])
    
    # Define a target orientation that rotates -90° about the x-axis.
    # This aligns the end-effector so that "forward" in the workspace corresponds
    # to the desired direction of the arm.
    target_orientation = np.array([
        [1,      0,       0],
        [0,      0,       1],
        [0,     -1,       0]
    ])
    
    # Set default initial guess to zeros if not provided.
    if initial_position is None:
        initial_position = [0] * len(braccio_chain.links)
    else:
        # If any value is > 2*pi, assume degrees and convert to radians.
        if max(initial_position) > 2 * math.pi:
            initial_position = [np.deg2rad(val) for val in initial_position]
    
    # Compute the inverse kinematics solution.
    ik_solution = braccio_chain.inverse_kinematics(
        target_position,
        target_orientation=target_orientation,
        orientation_mode="all",
        initial_position=initial_position
    )
    
    # Discard the first element (OriginLink) since it is fixed.
    joint_angles_rad = ik_solution[1:]
    joint_angles_deg = np.degrees(joint_angles_rad)
    
    # Optionally, clip the computed angles to their defined limits.
    limits_deg = [
        (0, 180),    # M1
        (15, 165),   # M2
        (0, 180),    # M3
        (0, 180)     # M4
    ]
    for i, (low, high) in enumerate(limits_deg):
        joint_angles_deg[i] = np.clip(joint_angles_deg[i], low, high)
    
    return joint_angles_deg
# ----------------------------------------------
# 3. Serial Communication Setup (COM5)
# ----------------------------------------------
try:
    ser = serial.Serial('COM5', 115200, timeout=1)
    time.sleep(2)  # allow Arduino to reset
except Exception as e:
    print("Error opening serial port:", e)
    ser = None

# ----------------------------------------------
# 4. Tkinter GUI with Embedded 3D Plot and Sliders/Entries
# ----------------------------------------------
root = tk.Tk()
root.title("Braccio Coordinate Controller")

# Create frames for controls and plot.
control_frame = ttk.Frame(root)
control_frame.grid(row=0, column=0, padx=10, pady=10)
plot_frame = ttk.Frame(root)
plot_frame.grid(row=0, column=1, padx=10, pady=10)

# --- Define slider ranges ---
# For target coordinates (meters)
x_min, x_max = 0.0, 0.3
y_min, y_max = -0.2, 0.2
z_min, z_max = 0.0, 0.3
# For delay (ms)
delay_min, delay_max = 10, 100
# For gripper (M6) in degrees.
m6_min, m6_max = 10, 73
# For coordinate offsets (meters)
offset_x_min, offset_x_max = -0.1, 0.1
offset_y_min, offset_y_max = -0.1, 0.1
offset_z_min, offset_z_max = -0.1, 0.1

# --- Tkinter Variables ---
x_val = tk.DoubleVar(value=0.15)
y_val = tk.DoubleVar(value=0.0)
z_val = tk.DoubleVar(value=0.0)
delay_val = tk.IntVar(value=20)
m6_val = tk.DoubleVar(value=10)
# New offset variables.
offset_x_val = tk.DoubleVar(value=0.0)
offset_y_val = tk.DoubleVar(value=0.0)
offset_z_val = tk.DoubleVar(value=0.0)

# --- Helper: Create slider with entry ---
def create_slider_with_entry(parent, label_text, var, min_val, max_val, row):
    ttk.Label(parent, text=label_text).grid(row=row, column=0, sticky="w")
    slider = ttk.Scale(parent, from_=min_val, to=max_val, variable=var, orient=tk.HORIZONTAL)
    slider.grid(row=row, column=1, padx=5, pady=5)
    entry = ttk.Entry(parent, textvariable=var, width=6)
    entry.grid(row=row, column=2, padx=5)
    return slider, entry

# Create sliders for target coordinates.
x_slider, x_entry = create_slider_with_entry(control_frame, "X (m)", x_val, x_min, x_max, row=0)
y_slider, y_entry = create_slider_with_entry(control_frame, "Y (m)", y_val, y_min, y_max, row=1)
z_slider, z_entry = create_slider_with_entry(control_frame, "Z (m)", z_val, z_min, z_max, row=2)
# Slider for delay.
delay_slider, delay_entry = create_slider_with_entry(control_frame, "Delay (ms)", delay_val, delay_min, delay_max, row=3)
# Slider for gripper (M6).
m6_slider, m6_entry = create_slider_with_entry(control_frame, "M6 (Grip °)", m6_val, m6_min, m6_max, row=4)
# Sliders for coordinate offset.
offset_x_slider, offset_x_entry = create_slider_with_entry(control_frame, "Offset X (m)", offset_x_val, offset_x_min, offset_x_max, row=5)
offset_y_slider, offset_y_entry = create_slider_with_entry(control_frame, "Offset Y (m)", offset_y_val, offset_y_min, offset_y_max, row=6)
offset_z_slider, offset_z_entry = create_slider_with_entry(control_frame, "Offset Z (m)", offset_z_val, offset_z_min, offset_z_max, row=7)


def adjust_target(x, y, z, rotation_angle_deg):
    # Rotate the target point around the y-axis by rotation_angle_deg
    theta = np.deg2rad(rotation_angle_deg)
    R = np.array([
        [np.cos(theta), 0, np.sin(theta)],
        [0,             1, 0],
        [-np.sin(theta), 0, np.cos(theta)]
    ])
    target = np.array([x, y, z])
    adjusted = R.dot(target)
    return adjusted


# Create a "Send Coordinates" button.
def send_coordinates():
    # Get current coordinate values.
    x = x_val.get()
    y = y_val.get()
    z = z_val.get()
    delay = delay_val.get()
    # Get current offset values.
    dx = offset_x_val.get()
    dy = offset_y_val.get()
    dz = offset_z_val.get()
    
    # Apply the offset.
    x_adj = x + dx
    y_adj = y + dy
    z_adj = z + dz

    x_adj, y_adj, z_adj = adjust_target(x_adj, y_adj, z_adj, 90)
    
    # Provide an initial guess for IK (in degrees, including fixed base).
    initial_guess_deg = [0, 0, 112, 0, 0]
    initial_guess = [np.deg2rad(val) for val in initial_guess_deg]  
    ik_angles = compute_braccio_angles(x_adj, y_adj, z_adj, initial_position=initial_guess)
    
    if ik_angles is None:
        status_label.config(text="IK solution not found!")
        return
    # Append M5 and M6 angles as 0 for now.
    ik_angles = np.append(ik_angles, 0)  # M5 (Wrist Rotation)
    ik_angles = np.append(ik_angles, 10)  # M6 (Gripper)
    # Override computed M6 with manually set value.
    ik_angles[5] = m6_val.get()
    # Format the command string: delay,angle1,...,angle6\n
    cmd = f"{delay}," + ",".join(f"{angle:.1f}" for angle in ik_angles) + "\n"
    if ser is not None:
        try:
            ser.write(cmd.encode('utf-8'))
            status_label.config(text=f"Sent: {cmd.strip()}")
        except Exception as e:
            status_label.config(text=f"Error sending: {e}")
    else:
        status_label.config(text="Serial port not available.")

send_button = ttk.Button(control_frame, text="Send Coordinates", command=send_coordinates)
send_button.grid(row=8, column=0, columnspan=3, pady=10)

status_label = ttk.Label(control_frame, text="Status: ")
status_label.grid(row=9, column=0, columnspan=3)

# -------------------------------
# 5. Matplotlib 3D Plot Embedded in Tkinter
# -------------------------------
fig = Figure(figsize=(5, 4), dpi=100)
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim(x_min, x_max)
ax.set_ylim(y_min, y_max)
ax.set_zlim(z_min, z_max)
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_zlabel("Z (m)")
# Plot a red point for the target.
target_point, = ax.plot([x_val.get()], [y_val.get()], [z_val.get()], 'ro')

canvas = FigureCanvasTkAgg(fig, master=plot_frame)
canvas.draw()
canvas.get_tk_widget().pack()

def update_plot(event=None):
    target_point.set_data([x_val.get()], [y_val.get()])
    target_point.set_3d_properties([z_val.get()])
    canvas.draw()

x_slider.bind("<ButtonRelease-1>", update_plot)
y_slider.bind("<ButtonRelease-1>", update_plot)
z_slider.bind("<ButtonRelease-1>", update_plot)

def periodic_update():
    update_plot()
    root.after(100, periodic_update)
periodic_update()

root.mainloop()

if ser is not None:
    ser.close()
