import tkinter as tk
import serial
import threading
import time

# Open serial port (adjust 'COM5' and baud rate if necessary)
ser = serial.Serial('COM5', 115200, timeout=0.5)
time.sleep(2)  # Give Arduino time to reset

# Global flag to indicate whether automation mode is active
automation_mode = False
last_command_time = time.time()  # Last time a command was sent

# Lock to control automation scheduling
automation_lock = threading.Lock()

def send_command():
    current = time.time()
    global last_command_time	
    if current - last_command_time < 0.1: # 100 ms delay between commands
        append_console("Command sent too quickly, skipping...\n")
        return
    last_command_time = current

    """Send the current command (step delay + servo angles) to the Arduino."""
    # Get current slider values
    step_delay = step_delay_slider.get()  # e.g., 20 ms
    m1 = slider_m1.get()
    m2 = slider_m2.get()
    m3 = slider_m3.get()
    m4 = slider_m4.get()
    m5 = slider_m5.get()
    m6 = slider_m6.get()
    # Construct a command string: "stepDelay,M1,M2,M3,M4,M5,M6\n"
    command = f"{step_delay},{m1},{m2},{m3},{m4},{m5},{m6}\n"
    try:
        ser.write(command.encode('utf-8'))
    except Exception as e:
        append_console(f"Error sending command: {e}\n")
    append_console(f"Sent: {command}")
    time.sleep(1)  # Give Arduino time to reset

def manual_slider_update(value):
    """Called when any manual slider is moved.
    If automation is not active, send the current command."""
    global automation_mode
    if not automation_mode:
        send_command()

def automation_send():
    """Sends command repeatedly at the interval set by the automation delay slider."""
    global automation_mode
    if automation_mode:
        send_command()
        # Get delay (in ms) from the automation delay slider
        delay_ms = automation_delay_slider.get()
        # Schedule the next automation call after delay_ms milliseconds
        root.after(delay_ms, automation_send)

def play_automation():
    """Activate automation mode so that commands are sent automatically."""
    global automation_mode
    automation_mode = True
    # Optionally disable manual slider update if desired
    append_console("Automation started.\n")
    automation_send()

def stop_automation():
    """Stop the automation mode."""
    global automation_mode
    automation_mode = False
    append_console("Automation stopped.\n")

def append_console(text):
    """Append text to the console widget and scroll to the end."""
    console_text.insert(tk.END, text)
    console_text.see(tk.END)

def read_serial():
    """Continuously read from the serial port and update the console."""
    while True:
        if ser.in_waiting:
            try:
                data = ser.readline().decode('utf-8', errors='ignore').strip()
                if data:
                    append_console(f"Received: {data}\n")
            except Exception as e:
                append_console(f"Error reading: {e}\n")
        time.sleep(0.1)

# Set up the Tkinter GUI
root = tk.Tk()
root.title("Braccio Arm Controller")

# Frame for servo and step delay sliders
slider_frame = tk.Frame(root)
slider_frame.pack(side=tk.TOP, fill=tk.BOTH, padx=10, pady=10)

# Slider for M1: Base (0-180 degrees)
slider_m1 = tk.Scale(slider_frame, from_=0, to=180, orient=tk.HORIZONTAL, label="M1 (Base)", command=manual_slider_update)
slider_m1.set(90)  # Default 90°
slider_m1.pack(fill=tk.X, pady=5)

# Slider for M2: Shoulder (15-165 degrees)
slider_m2 = tk.Scale(slider_frame, from_=15, to=165, orient=tk.HORIZONTAL, label="M2 (Shoulder)", command=manual_slider_update)
slider_m2.set(45)
slider_m2.pack(fill=tk.X, pady=5)

# Slider for M3: Elbow (0-180 degrees)
slider_m3 = tk.Scale(slider_frame, from_=0, to=180, orient=tk.HORIZONTAL, label="M3 (Elbow)", command=manual_slider_update)
slider_m3.set(180)
slider_m3.pack(fill=tk.X, pady=5)

# Slider for M4: Wrist Vertical (0-180 degrees)
slider_m4 = tk.Scale(slider_frame, from_=0, to=180, orient=tk.HORIZONTAL, label="M4 (Wrist Vertical)", command=manual_slider_update)
slider_m4.set(180)
slider_m4.pack(fill=tk.X, pady=5)

# Slider for M5: Wrist Rotation (0-180 degrees)
slider_m5 = tk.Scale(slider_frame, from_=0, to=180, orient=tk.HORIZONTAL, label="M5 (Wrist Rotation)", command=manual_slider_update)
slider_m5.set(90)
slider_m5.pack(fill=tk.X, pady=5)

# Slider for M6: Gripper (10-73 degrees)
slider_m6 = tk.Scale(slider_frame, from_=10, to=73, orient=tk.HORIZONTAL, label="M6 (Gripper)", command=manual_slider_update)
slider_m6.set(10)
slider_m6.pack(fill=tk.X, pady=5)

# Slider for Step Delay (e.g., 10-30 ms)
step_delay_slider = tk.Scale(slider_frame, from_=10, to=30, orient=tk.HORIZONTAL, label="Step Delay (ms)", command=manual_slider_update)
step_delay_slider.set(20)
step_delay_slider.pack(fill=tk.X, pady=5)

# Frame for automation controls
automation_frame = tk.Frame(root)
automation_frame.pack(side=tk.TOP, fill=tk.X, padx=10, pady=10)

# Slider for Automation Delay (how often to send command, in milliseconds)
automation_delay_slider = tk.Scale(automation_frame, from_=500, to=5000, orient=tk.HORIZONTAL, label="Automation Delay (ms)")
automation_delay_slider.set(2000)
automation_delay_slider.pack(fill=tk.X, pady=5)

# Play and Stop buttons for automation mode
play_button = tk.Button(automation_frame, text="Play", command=play_automation)
play_button.pack(side=tk.LEFT, padx=5)

stop_button = tk.Button(automation_frame, text="Stop", command=stop_automation)
stop_button.pack(side=tk.LEFT, padx=5)

# Console text widget to display serial logs
console_text = tk.Text(root, height=10, wrap=tk.WORD)
console_text.pack(fill=tk.BOTH, padx=10, pady=10, expand=True)

# Start a thread to continuously read from the serial port
serial_thread = threading.Thread(target=read_serial, daemon=True)
serial_thread.start()

root.mainloop()
