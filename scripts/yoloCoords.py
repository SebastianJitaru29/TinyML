import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO
import math

model = YOLO("yolov8n.pt") #https://github.com/ultralytics/assets/releases/download/v8.1.0/yolov8n.pt

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

profile = pipeline.start(config)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale:", depth_scale)

# Create an align object so that depth frames are aligned to color frames.
align_to = rs.stream.color
align = rs.align(align_to)


# For a top-down view (easiest), if the camera is rotated by -90° about its X-axis relative to the robot frame,
# a corresponding rotation matrix R can be defined. For example:
R = np.array([
    [1,  0,  0],
    [0,  0,  1],
    [0, -1,  0]
])
# Translation vector t (in meters). Adjust these values after calibration.
# Example: if the camera is mounted 0.5 m above the robot base.
t = np.array([0, 0, 0.5])

def transform_point(point_cam):
    """
    Transforms a 3D point from camera coordinates to robot coordinates.
    point_cam: 3-element array or list from deprojection (in meters)
    Returns a 3-element numpy array in robot coordinates.
    """
    point_cam = np.array(point_cam)
    point_robot = R.dot(point_cam) + t
    return point_robot

# If the arm operates in the horizontal plane (X-Y), with the robot's base at (0,0,0).
# We'll compute:
#   - M1: base rotation (angle between robot X axis and the target's projection)
#   - M2: shoulder angle
#   - M3: elbow angle
#
# Define link lengths in meters. Adjust these according to your arm's dimensions.
L1 = 0.1  # length of first link (shoulder)
L2 = 0.1  # length of second link (forearm/elbow)

def compute_ik(target):
    """
    Compute a simple IK solution for a 2-link planar arm.
    target: 3D point (X, Y, Z) in robot coordinates.
    Returns: (base_angle, shoulder_angle, elbow_angle) in degrees, or None if unreachable.
    Note: Only the X and Y coordinates are used for the planar solution.
    """
    Xr, Yr, Zr = target
    # Base rotation: angle from robot X axis to the target projection in the X-Y plane.
    base_angle = math.degrees(math.atan2(Yr, Xr))
    
    # Distance from the base to the target in the horizontal plane.
    d = math.sqrt(Xr**2 + Yr**2)
    
    # Check reachability: if d is outside [|L1-L2|, L1+L2], the target is unreachable.
    if d > (L1 + L2) or d < abs(L1 - L2):
        print("Target out of reach: distance =", d)
        return None

    # Law of Cosines to compute the elbow angle.
    cos_angle = (L1**2 + L2**2 - d**2) / (2 * L1 * L2)
    elbow_angle_rad = math.acos(np.clip(cos_angle, -1.0, 1.0))
    elbow_angle = math.degrees(elbow_angle_rad)
    
    # Compute the shoulder angle.
    angle_to_target = math.atan2(Yr, Xr)
    cos_angle_shoulder = (L1**2 + d**2 - L2**2) / (2 * L1 * d)
    shoulder_offset = math.acos(np.clip(cos_angle_shoulder, -1.0, 1.0))
    shoulder_angle_rad = angle_to_target - shoulder_offset
    shoulder_angle = math.degrees(shoulder_angle_rad)
    
    return base_angle, shoulder_angle, elbow_angle

try:
    while True:
        # Wait for a new set of frames
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Run YOLO detection on the color image
        results = model(color_image, verbose=False)[0]
        
        # Retrieve the intrinsics for deprojection
        intrinsics = color_frame.profile.as_video_stream_profile().intrinsics

        for box in results.boxes:
            # Get bounding box coordinates (xyxy format)
            coords = box.xyxy[0].cpu().numpy()
            x1, y1, x2, y2 = coords.astype(int)
            cls = int(box.cls[0])
            conf = float(box.conf[0])
            
            # Compute center of bounding box
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)
            
            # Retrieve depth at center (in meters)
            depth = depth_frame.get_distance(cx, cy)
            if depth == 0:
                continue  # skip if no valid depth
            
            # Deproject the 2D pixel (cx,cy) into a 3D point (camera coordinates)
            point_cam = rs.rs2_deproject_pixel_to_point(intrinsics, [cx, cy], depth)
            
            # Transform the point from camera to robot coordinates
            point_robot = transform_point(point_cam)
            
            # Compute inverse kinematics (IK) to get joint angles (for M1, M2, M3)
            ik_angles = compute_ik(point_robot)
            if ik_angles is not None:
                base_angle, shoulder_angle, elbow_angle = ik_angles
                ik_text = f"IK: M1={base_angle:.1f}, M2={shoulder_angle:.1f}, M3={elbow_angle:.1f}"
                cv2.putText(color_image, ik_text, (x1, y1 - 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                print(f"Detection (class {cls}) at 3D {point_robot}: {ik_text}")
            else:
                print("IK not solvable for detected point:", point_robot)
            
            # Draw bounding box and label on the image
            label = f"Cls:{cls} {conf:.2f} Depth:{depth:.2f}m"
            cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(color_image, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Optionally, draw the robot coordinates on the image
            coord_text = f"R:({point_robot[0]:.2f},{point_robot[1]:.2f},{point_robot[2]:.2f})"
            cv2.putText(color_image, coord_text, (x1, y1 - 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # Display the output image
        cv2.imshow("YOLO Detections and IK", color_image)
        if cv2.waitKey(1) == 27:  # ESC key to exit
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
