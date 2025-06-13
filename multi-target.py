import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
import time
import math
import os
import sys

# --- Import and Verify Drone Control Functions ---
try:
    from drone_control_functions import (
        connect_to_vehicle, arm_vehicle, takeoff, set_mode,
        rotate_yaw, land_vehicle, disarm_vehicle,
        set_body_ned_velocity, # Import the new velocity function
        master, mavutil
    )
except ImportError:
    print("Error: Make sure 'drone_control.py' is updated and in the same directory.")
    sys.exit(1)

# --- Mission Configuration ---
# Models
SAFE_SPOT_MODEL_PATH = 'files_cv/train17/weights/best.pt'
SLANT_SPOT_MODEL_PATH = 'files_cv/train20/weights/best.pt'

# Camera
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FPS = 30

# Flight Parameters
INITIAL_TAKEOFF_ALTITUDE = 1.5
SUBSEQUENT_TAKEOFF_ALTITUDE = 1.0
ROTATION_ANGLE_STEP = 5
ROTATION_SPEED = 10
SEARCH_ROTATION_ANGLE = 25
CONFIDENCE_THRESHOLD = 0.60

# --- NEW: Precision Approach Parameters ---
LANDING_ZONE_DISTANCE = 0.25  # Land when within 25cm of the target.
PROPORTIONAL_GAIN = 0.4       # Controls how aggressively the drone seeks the target.
MAX_APPROACH_SPEED = 0.3      # Max forward speed in m/s during approach.

# Suppress YOLO verbose output
os.environ['YOLO_VERBOSE'] = 'False'

# --- (The following helper functions remain the same) ---

def initialize_camera():
    """Sets up and starts the RealSense camera pipeline."""
    print("Initializing RealSense camera...")
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, FRAME_WIDTH, FRAME_HEIGHT, rs.format.bgr8, FPS)
    config.enable_stream(rs.stream.depth, FRAME_WIDTH, FRAME_HEIGHT, rs.format.z16, FPS)
    colorizer = rs.colorizer()
    colorizer.set_option(rs.option.color_scheme, 8)
    align = rs.align(rs.stream.color)
    pipeline.start(config)
    print("Camera initialized.")
    return pipeline, align, colorizer

def classify_centroid_position(cx, frame_width, tolerance_percent=8):
    """Classifies if the centroid is Left, Center, or Right."""
    mid_x = frame_width / 2
    tolerance_pixels = frame_width * (tolerance_percent / 100)
    if cx < mid_x - tolerance_pixels: return "Left"
    if cx > mid_x + tolerance_pixels: return "Right"
    return "Center"

def get_best_detection(model, blended_image):
    """Finds the detection with the highest confidence in a frame."""
    results = model(blended_image, verbose=False)
    best_detection, max_conf = None, 0
    for r in results:
        for box in r.boxes:
            conf = float(box.conf[0])
            if conf > max_conf:
                max_conf, best_detection = conf, box
    if best_detection and max_conf > CONFIDENCE_THRESHOLD:
        x1, y1, x2, y2 = map(int, best_detection.xyxy[0])
        return best_detection, ((x1 + x2) // 2, (y1 + y2) // 2), max_conf
    return None, None, 0

# --- NEW: Precision Approach Function ---

def execute_precision_approach(mission_name, camera_comps, model):
    """
    Executes a closed-loop approach using proportional control.
    Returns True on success, False on failure.
    """
    print("\n--- Starting Precision Approach ---")
    pipeline, align, colorizer = camera_comps

    while True:
        # --- 1. Get Camera Data ---
        frames = pipeline.wait_for_frames(timeout_ms=5000) # Wait for a frame
        aligned_frames = align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        if not depth_frame or not color_frame:
            print("Warning: Missed a frame.")
            continue
        
        # --- 2. Process Image ---
        color_image = np.asanyarray(color_frame.get_data())
        depth_heatmap = np.asanyarray(colorizer.colorize(depth_frame).get_data())
        blended_image = cv2.addWeighted(color_image, 0.75, depth_heatmap, 0.15, 0)
        
        detection, centroid, conf = get_best_detection(model, blended_image)

        # --- 3. Control Logic ---
        if detection:
            cx, cy = centroid
            current_distance = depth_frame.get_distance(cx, cy)
            
            # Annotate for visual feedback
            pos_tag = classify_centroid_position(cx, FRAME_WIDTH)
            cv2.putText(blended_image, f"Dist: {current_distance:.2f}m | Pos: {pos_tag}", 
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.imshow(f"Mission: {mission_name}", blended_image)

            # Check if we have arrived
            if 0 < current_distance <= LANDING_ZONE_DISTANCE:
                print(f"Success! Reached landing zone at {current_distance:.2f}m.")
                set_body_ned_velocity(0, 0, 0) # STOP all movement
                return True

            # If not arrived, adjust movement
            if pos_tag == "Center":
                # Proportional control for forward speed
                forward_speed = min(current_distance * PROPORTIONAL_GAIN, MAX_APPROACH_SPEED)
                print(f"Target centered. Moving forward at {forward_speed:.2f} m/s.")
                set_body_ned_velocity(forward_speed, 0, 0)
            else:
                # Stop forward movement to correct heading
                print("Target off-center. Pausing to rotate.")
                set_body_ned_velocity(0, 0, 0)
                # Make small corrective rotation
                direction = True if pos_tag == "Right" else False
                rotate_yaw(2, ROTATION_SPEED, clockwise=direction)
        else:
            # Target lost during approach
            print("Target lost during approach! Stopping.")
            set_body_ned_velocity(0, 0, 0)
            cv2.imshow(f"Mission: {mission_name}", blended_image)
            return False # Signal failure

        if cv2.waitKey(1) & 0xFF == ord('q'):
            raise KeyboardInterrupt("User quit.")
            
# --- REVISED: Main Mission Logic ---

def execute_find_and_land_mission(mission_name, model, camera_comps, should_takeoff_again=False):
    """
    Revised mission to use the new precision approach logic.
    """
    print(f"\n{'='*15} STARTING MISSION: {mission_name.upper()} {'='*15}")
    pipeline, align, colorizer = camera_comps

    # --- Phase 1: Search and Center ---
    # (This part remains largely the same, it just finds the target initially)
    # ... code for searching ...

    # --- Phase 1: Search and Center on Target ---
    while True:
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        if not color_frame or not depth_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())
        depth_heatmap = np.asanyarray(colorizer.colorize(depth_frame).get_data())
        blended_image = cv2.addWeighted(color_image, 0.75, depth_heatmap, 0.15, 0)
        
        detection, centroid, conf = get_best_detection(model, blended_image)
        
        position_tag = None
        if detection:
            cx, cy = centroid
            position_tag = classify_centroid_position(cx, FRAME_WIDTH)
            # Annotate frame for visualization
            x1, y1, x2, y2 = map(int, detection.xyxy[0])
            cv2.rectangle(blended_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(blended_image, centroid, 5, (0, 255, 255), -1)
            cv2.putText(blended_image, f"{position_tag} ({conf:.2f})", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        cv2.imshow(f"Mission: {mission_name}", blended_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            raise KeyboardInterrupt("User quit.")

        if position_tag == "Center":
            distance = depth_frame.get_distance(centroid[0], centroid[1])
            if distance > 0.1: # Sanity check for valid distance
                print(f"Target Centered! Distance: {distance:.2f} meters.")
                target_distance = distance
                break
            else:
                print("Centering complete, but distance is zero. Re-searching...")
        elif position_tag == "Left":
            print("Target is Left. Rotating Counter-Clockwise.")
            rotate_yaw(ROTATION_ANGLE_STEP, ROTATION_SPEED, clockwise=False)
        elif position_tag == "Right":
            print("Target is Right. Rotating Clockwise.")
            rotate_yaw(ROTATION_ANGLE_STEP, ROTATION_SPEED, clockwise=True)
        else:
            print("No target found. Searching...")
            rotate_yaw(SEARCH_ROTATION_ANGLE, ROTATION_SPEED, clockwise=True)
    
    # --- Phase 2: Approach (Calls the new function) ---
    approach_successful = execute_precision_approach(mission_name, camera_comps, model)

    # --- Phase 3: Land and Optional Takeoff ---
    if approach_successful:
        print("Landing on target.")
        land_vehicle()
        disarm_vehicle()
        if should_takeoff_again:
            print("\nTaking off for next mission phase...")
            arm_vehicle()
            takeoff(SUBSEQUENT_TAKEOFF_ALTITUDE)
            set_mode('GUIDED')
        print(f"\n{'='*15} MISSION COMPLETE: {mission_name.upper()} {'='*15}")
    else:
        print("Mission phase failed. Landing at current location.")
        land_vehicle()
        disarm_vehicle()
        raise SystemExit(f"Mission '{mission_name}' aborted due to failed approach.")

# --- (Main orchestrator remains the same) ---
def main():
    """Main function to initialize and run the sequence of missions."""
    connect_to_vehicle()
    if master is None: sys.exit("Failed to connect to vehicle.")
        
    safe_spot_model = YOLO(SAFE_SPOT_MODEL_PATH)
    slant_spot_model = YOLO(SLANT_SPOT_MODEL_PATH)
    camera_components = initialize_camera()

    try:
        arm_vehicle()
        takeoff(INITIAL_TAKEOFF_ALTITUDE)
        set_mode('GUIDED')
        
        execute_find_and_land_mission(
            mission_name="Primary Safe Spot",
            model=safe_spot_model,
            camera_comps=camera_components,
            should_takeoff_again=True
        )
        
        execute_find_and_land_mission(
            mission_name="Slant Safe Spot",
            model=slant_spot_model,
            camera_comps=camera_components,
            should_takeoff_again=False
        )
        print("\nAll mission objectives accomplished!")

    except (KeyboardInterrupt, SystemExit) as e:
        print(f"\nMission terminated: {e}. Attempting to land safely.")
        land_vehicle()
    except Exception as e:
        print(f"\nAn unexpected error occurred: {e}")
        print("Attempting to land safely as a precaution.")
        land_vehicle()
    finally:
        print("Cleaning up resources.")
        camera_components[0].stop()
        cv2.destroyAllWindows()
        print("Script finished.")


if __name__ == "__main__":
    main()
