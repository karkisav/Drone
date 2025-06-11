import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
import time
import math
import os
import sys

# --- Import drone control functions from your existing script ---
# This assumes 'drone_control.py' is in the same directory.
try:
    from drone_control import (
        connect_to_vehicle,
        arm_vehicle,
        takeoff,
        set_mode,
        rotate_yaw,
        move_body_ned,
        land_vehicle,
        master, # Import the master object to use it for speed commands
        mavutil # Import mavutil for the speed command
    )
except ImportError:
    print("Error: Make sure 'drone_control.py' and its dependencies are available.")
    sys.exit(1)

# --- Configuration ---
# CV Model and Camera
SAFE_SPOT_MODEL_PATH = 'files_cv/train17/weights/best.pt'
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FPS = 30

# Flight Parameters
TAKEOFF_ALTITUDE = 1.5  # meters
ROTATION_ANGLE_STEP = 5  # degrees for centering
ROTATION_SPEED = 10  # degrees per second
SEARCH_ROTATION_ANGLE = 20 # degrees to turn if no spot is found
FORWARD_SPEED = 0.25 # m/s
DETECTION_CONFIDENCE_THRESHOLD = 0.50 # Minimum confidence to trust a detection

# Suppress YOLO verbose output
os.environ['YOLO_VERBOSE'] = 'False'

# --- Computer Vision Functions ---

def classify_centroid_position(cx, frame_width, tolerance_percent=10):
    """Classifies if the centroid is Left, Center, or Right in the frame."""
    mid_x = frame_width / 2
    tolerance_pixels = frame_width * (tolerance_percent / 100)

    if cx < mid_x - tolerance_pixels:
        return "Left"
    elif cx > mid_x + tolerance_pixels:
        return "Right"
    else:
        return "Center"

def find_and_analyze_safe_spot(pipeline, align, model, frame_width, colorizer):
    """
    Captures frames, creates a blended RGB-Depth image, finds the most
    confident 'safe_spot', and returns its position, distance, and an
    annotated image for visualization.
    """
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    color_frame = aligned_frames.get_color_frame()
    depth_frame = aligned_frames.get_depth_frame()

    if not color_frame or not depth_frame:
        return None, None, None  # No frame data

    # --- MODIFICATION: Create the blended image for YOLO ---
    # 1. Convert frames to NumPy arrays
    color_image = np.asanyarray(color_frame.get_data())

    # 2. Create the colorized depth heatmap
    colorized_depth = colorizer.colorize(depth_frame)
    depth_heatmap = np.asanyarray(colorized_depth.get_data())

    # 3. Create the blended image, same as your original script
    blended_image = cv2.addWeighted(color_image, 0.75, depth_heatmap, 0.15, 0)
    # --- END MODIFICATION ---

    # 4. Run detection on the correct (blended) image
    results = model(blended_image, verbose=False)

    best_detection = None
    max_conf = 0.0

    # Find the detection with the highest confidence
    for r in results:
        for box in r.boxes:
            conf = float(box.conf[0])
            if conf > max_conf:
                max_conf = conf
                best_detection = box

    # If a detection exceeds our confidence threshold, process it
    if best_detection and best_detection.conf[0] > DETECTION_CONFIDENCE_THRESHOLD:
        x1, y1, x2, y2 = map(int, best_detection.xyxy[0])
        cx = (x1 + x2) // 2
        cy = (y1 + y2) // 2

        position_tag = classify_centroid_position(cx, frame_width)
        # IMPORTANT: Get distance from the original, raw depth frame
        distance_m = depth_frame.get_distance(cx, cy)

        # Create an annotated frame for display using the blended image as a base
        annotated_frame = blended_image.copy()
        cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.circle(annotated_frame, (cx, cy), 5, (0, 255, 255), -1)
        label_text = f"Tag: {position_tag}, Dist: {distance_m:.2f}m"
        cv2.putText(annotated_frame, label_text, (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        return position_tag, distance_m, annotated_frame

    # Return the unannotated blended image if no good detection is found
    return None, None, blended_image


# --- Main Flight Control Logic ---
def main():
    """
    Main function to execute the intelligent flight mission.
    """
    # --- Initialization ---
    print("Initializing...")
    connect_to_vehicle() # Connects and sets the global 'master' object
    if master is None:
        print("Failed to connect to vehicle. Exiting.")
        sys.exit(1)
        
    safe_spot_model = YOLO(SAFE_SPOT_MODEL_PATH)

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, FRAME_WIDTH, FRAME_HEIGHT, rs.format.bgr8, FPS)
    config.enable_stream(rs.stream.depth, FRAME_WIDTH, FRAME_HEIGHT, rs.format.z16, FPS)
    
    # --- MODIFICATION: Initialize the colorizer for blending ---
    colorizer = rs.colorizer()
    colorizer.set_option(rs.option.color_scheme, 8) # 'Jet' color scheme
    # --- END MODIFICATION ---

    align = rs.align(rs.stream.color)
    
    pipeline.start(config)
    print("Camera and Drone Connection Initialized.")

    try:
        # --- Pre-flight Sequence ---
        arm_vehicle()
        takeoff(TAKEOFF_ALTITUDE)
        set_mode('GUIDED')

        print("\n--- Phase 1: Searching for Safe Spot ---")
        target_distance = None
        
        while True:
            # Pass the colorizer to the analysis function
            position, distance, frame = find_and_analyze_safe_spot(
                pipeline, align, safe_spot_model, FRAME_WIDTH, colorizer
            )
            
            # Display the camera feed (now showing the blended view)
            if frame is not None:
                cv2.imshow("Live Analysis", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                raise KeyboardInterrupt # Allow manual exit

            if position == "Center":
                print(f"Target Centered! Distance: {distance:.2f} meters.")
                # Ensure distance is valid before breaking loop
                if distance > 0.1: # Basic sanity check
                    target_distance = distance
                    break
                else:
                    print("Centering complete, but distance is zero. Re-searching...")
            elif position == "Left":
                print("Target is Left. Rotating Counter-Clockwise.")
                rotate_yaw(ROTATION_ANGLE_STEP, ROTATION_SPEED, clockwise=False)
            elif position == "Right":
                print("Target is Right. Rotating Clockwise.")
                rotate_yaw(ROTATION_ANGLE_STEP, ROTATION_SPEED, clockwise=True)
            else:
                print("No target found. Searching...")
                rotate_yaw(SEARCH_ROTATION_ANGLE, ROTATION_SPEED, clockwise=True)
            
            time.sleep(0.5) # Pause between actions

        # --- Phase 2: Approach Maneuver ---
        if target_distance:
            # Per your request, calculate move distance: dist * cos(30)
            move_dist = target_distance * math.cos(math.radians(30))
            
            print(f"\n--- Phase 2: Approaching Target ---")
            print(f"Calculated move distance: {move_dist:.2f} meters.")
            
            # Set forward speed
            master.mav.command_long_send(
                master.target_system, master.target_component,
                mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0,
                1, FORWARD_SPEED, -1, 0, 0, 0, 0
            )
            time.sleep(1)

            move_body_ned(move_dist)
            print("Approach complete.")
        else:
            print("Could not determine valid distance. Skipping approach.")

        # --- Phase 3: Landing ---
        print("\n--- Phase 3: Landing ---")
        land_vehicle()
        print("Mission accomplished.")

    except KeyboardInterrupt:
        print("\nMission interrupted by user. Landing immediately.")
        land_vehicle()
    except Exception as e:
        print(f"\nAn error occurred: {e}")
        print("Attempting to land safely.")
        land_vehicle()
    finally:
        # --- Cleanup ---
        print("Stopping camera and closing windows.")
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()