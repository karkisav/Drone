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
    from drone_control import (
        connect_to_vehicle, arm_vehicle, takeoff, set_mode,
        rotate_yaw, move_body_ned, land_vehicle, disarm_vehicle,
        master, mavutil
    )
except ImportError:
    print("Error: Make sure 'drone_control.py' is updated and in the same directory.")
    sys.exit(1)

# --- Mission Configuration ---
# Models
SAFE_SPOT_MODEL_PATH = 'files_cv/train17/weights/best.pt'
SLANT_SPOT_MODEL_PATH = 'files_cv/train20/weights/best.pt' # Your second model

# Camera
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FPS = 30

# Flight Parameters
INITIAL_TAKEOFF_ALTITUDE = 1.5  # meters
SUBSEQUENT_TAKEOFF_ALTITUDE = 1.0 # meters for taking off after the first landing
ROTATION_ANGLE_STEP = 5       # degrees for fine-tuning aim
ROTATION_SPEED = 10           # degrees per second
SEARCH_ROTATION_ANGLE = 90    # degrees to turn if no target is found
FORWARD_SPEED = 0.25          # m/s
CONFIDENCE_THRESHOLD = 0.60   # Minimum confidence to trust a detection

# Suppress YOLO verbose output
os.environ['YOLO_VERBOSE'] = 'False'


# --- Modular Computer Vision Functions ---

def initialize_camera():
    """Sets up and starts the RealSense camera pipeline."""
    print("Initializing RealSense camera...")
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, FRAME_WIDTH, FRAME_HEIGHT, rs.format.bgr8, FPS)
    config.enable_stream(rs.stream.depth, FRAME_WIDTH, FRAME_HEIGHT, rs.format.z16, FPS)
    
    colorizer = rs.colorizer()
    colorizer.set_option(rs.option.color_scheme, 8) # Jet color scheme
    
    align = rs.align(rs.stream.color)
    
    profile = pipeline.start(config)
    print("Camera initialized.")
    return pipeline, align, colorizer

def classify_centroid_position(cx, frame_width, tolerance_percent=10):
    """Classifies if the centroid is Left, Center, or Right."""
    mid_x = frame_width / 2
    tolerance_pixels = frame_width * (tolerance_percent / 100)
    if cx < mid_x - tolerance_pixels: return "Left"
    if cx > mid_x + tolerance_pixels: return "Right"
    return "Center"

def get_best_detection(model, blended_image):
    """
    Finds the detection with the highest confidence in a frame.
    Returns: Bounding box object, centroid (cx, cy), and confidence.
    """
    results = model(blended_image, verbose=False)
    best_detection = None
    max_conf = 0
    
    for r in results:
        for box in r.boxes:
            conf = float(box.conf[0])
            if conf > max_conf:
                max_conf = conf
                best_detection = box
                
    if best_detection and max_conf > CONFIDENCE_THRESHOLD:
        x1, y1, x2, y2 = map(int, best_detection.xyxy[0])
        cx = (x1 + x2) // 2
        cy = (y1 + y2) // 2
        return best_detection, (cx, cy), max_conf
        
    return None, None, 0


# --- Reusable Mission Logic ---

def execute_find_and_land_mission(mission_name, model, camera_comps, should_takeoff_again=False):
    """
    A complete, modular mission to find, approach, and land on a target.
    
    Args:
        mission_name (str): Name for this mission phase (e.g., "Primary Safe Spot").
        model (YOLO): The trained YOLO model to use for detection.
        camera_comps (tuple): Contains the pipeline, aligner, and colorizer.
        should_takeoff_again (bool): If True, drone will take off after landing.
    """
    print(f"\n{'='*15} STARTING MISSION: {mission_name.upper()} {'='*15}")
    pipeline, align, colorizer = camera_comps
    target_distance = None

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
            
    # --- Phase 2: Approach ---
    if target_distance:
        move_dist = target_distance * math.cos(math.radians(30))
        print(f"Approaching target. Moving forward {move_dist:.2f} meters.")
        move_body_ned(move_dist)
        print("Approach complete.")
    else:
        print("Could not get a valid distance. Aborting approach.")
        # Decide what to do here - maybe land or search again
        land_vehicle()
        raise SystemExit("Mission failed: Could not lock target distance.")

    # --- Phase 3: Land and Optional Takeoff ---
    print("Landing on target.")
    land_vehicle()
    disarm_vehicle()

    if should_takeoff_again:
        print("\nTaking off for next mission phase...")
        arm_vehicle()
        takeoff(SUBSEQUENT_TAKEOFF_ALTITUDE)
        set_mode('GUIDED')
    
    print(f"\n{'='*15} MISSION COMPLETE: {mission_name.upper()} {'='*15}")


# --- Main Script Orchestrator ---
def main():
    """Main function to initialize and run the sequence of missions."""
    # --- Initialization ---
    connect_to_vehicle()
    if master is None: sys.exit("Failed to connect to vehicle.")
        
    safe_spot_model = YOLO(SAFE_SPOT_MODEL_PATH)
    slant_spot_model = YOLO(SLANT_SPOT_MODEL_PATH)
    camera_components = initialize_camera()

    try:
        # --- Initial Flight Setup ---
        arm_vehicle()
        takeoff(INITIAL_TAKEOFF_ALTITUDE)
        set_mode('GUIDED')
        
        # --- EXECUTE MISSION 1 ---
        execute_find_and_land_mission(
            mission_name="Primary Safe Spot",
            model=safe_spot_model,
            camera_comps=camera_components,
            should_takeoff_again=True
        )
        
        # --- EXECUTE MISSION 2 ---
        execute_find_and_land_mission(
            mission_name="Slant Safe Spot",
            model=slant_spot_model,
            camera_comps=camera_components,
            should_takeoff_again=False # This is the final landing
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
        camera_components[0].stop() # Stop the pipeline
        cv2.destroyAllWindows()
        print("Script finished.")


if __name__ == "__main__":
    main()
