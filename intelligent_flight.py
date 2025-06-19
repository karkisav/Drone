import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
import time
import math
import os
import sys

# [--- MODIFIED ---]
# Import the new, cleaner functions from our robust drone_control library.
# Note that we removed functions we no longer need, like move_body_ned.
try:
    from drone_control import (
        connect_to_vehicle,
        set_mode,
        arm_vehicle,
        takeoff,
        rotate_yaw,
        send_body_ned_velocity,  # Use this for all velocity commands
        land_vehicle,
        master,
        mavutil,
    )
except ImportError:
    print("Error: Make sure 'drone_control.py' is in the same directory.")
    sys.exit(1)

# --- Configuration (Unchanged) ---
SAFE_SPOT_MODEL_PATH = "files_cv/train17/weights/best.pt"
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FPS = 30
TAKEOFF_ALTITUDE = 1.5
ROTATION_ANGLE_STEP = 5
ROTATION_SPEED = 10
SEARCH_ROTATION_ANGLE = 20
DETECTION_CONFIDENCE_THRESHOLD = 0.50
PROPORTIONAL_GAIN = 0.4
APPROACH_MAX_SPEED = 0.3
STOPPING_DISTANCE = 0.5
os.environ["YOLO_VERBOSE"] = "False"


# --- Computer Vision Functions (Unchanged) ---
def classify_centroid_position(cx, frame_width, tolerance_percent=10):
    mid_x = frame_width / 2
    tolerance_pixels = frame_width * (tolerance_percent / 100)
    if cx < mid_x - tolerance_pixels:
        return "Left"
    elif cx > mid_x + tolerance_pixels:
        return "Right"
    else:
        return "Center"


def find_and_analyze_safe_spot(pipeline, align, model, colorizer):
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    color_frame = aligned_frames.get_color_frame()
    depth_frame = aligned_frames.get_depth_frame()
    if not color_frame or not depth_frame:
        return None, None, None
    color_image = np.asanyarray(color_frame.get_data())
    colorized_depth = colorizer.colorize(depth_frame)
    depth_heatmap = np.asanyarray(colorized_depth.get_data())
    blended_image = cv2.addWeighted(color_image, 0.75, depth_heatmap, 0.15, 0)
    results = model(blended_image, verbose=False)
    best_detection = None
    max_conf = 0.0
    for r in results:
        for box in r.boxes:
            conf = float(box.conf[0])
            if conf > max_conf:
                max_conf = conf
                best_detection = box
    if best_detection and best_detection.conf[0] > DETECTION_CONFIDENCE_THRESHOLD:
        x1, y1, x2, y2 = map(int, best_detection.xyxy[0])
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
        position_tag = classify_centroid_position(cx, FRAME_WIDTH)
        distance_m = depth_frame.get_distance(cx, cy)
        annotated_frame = blended_image.copy()
        cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.circle(annotated_frame, (cx, cy), 5, (0, 255, 255), -1)
        label_text = f"Tag: {position_tag}, Dist: {distance_m:.2f}m"
        cv2.putText(
            annotated_frame,
            label_text,
            (x1, y1 - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 255),
            2,
        )
        return position_tag, distance_m, annotated_frame
    return None, None, blended_image


# --- [--- MODIFIED ---] Closed-Loop Control Functions ---

# We DELETED the local send_ned_velocity function because it's now in drone_control.py


def proportional_approach(pipeline, align, model, colorizer):
    print("\n--- Phase 2: Executing Proportional Approach ---")
    loop_hz = 10
    while True:
        position, distance, frame = find_and_analyze_safe_spot(
            pipeline, align, model, colorizer
        )
        if frame is not None:
            cv2.imshow("Live Analysis", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            raise KeyboardInterrupt

        if distance is None or distance == 0:
            print("Target lost. Hovering.")
            send_body_ned_velocity(0, 0, 0)  # Use the imported function
            time.sleep(1 / loop_hz)
            continue

        if position != "Center":
            print(f"Target drifted {position}. Re-centering.")
            send_body_ned_velocity(0, 0, 0)  # Stop forward movement
            is_clockwise = position == "Right"
            rotate_yaw(ROTATION_ANGLE_STEP, ROTATION_SPEED, clockwise=is_clockwise)
            continue

        error = distance - STOPPING_DISTANCE
        if error <= 0:
            print(f"Target reached. Final distance: {distance:.2f}m. Stopping.")
            send_body_ned_velocity(0, 0, 0)
            break

        speed = min(error * PROPORTIONAL_GAIN, APPROACH_MAX_SPEED)
        print(f"Dist: {distance:.2f}m, Error: {error:.2f}m, Speed: {speed:.2f} m/s")
        send_body_ned_velocity(speed, 0, 0)  # Use the imported function
        time.sleep(1 / loop_hz)
    print("Approach complete.")


# --- Main Flight Control Logic (Now cleaner) ---
def main():
    print("Initializing...")
    connect_to_vehicle()
    if master is None:
        sys.exit("Failed to connect to vehicle.")

    safe_spot_model = YOLO(SAFE_SPOT_MODEL_PATH)
    pipeline, config = rs.pipeline(), rs.config()
    config.enable_stream(
        rs.stream.color, FRAME_WIDTH, FRAME_HEIGHT, rs.format.bgr8, FPS
    )
    config.enable_stream(rs.stream.depth, FRAME_WIDTH, FRAME_HEIGHT, rs.format.z16, FPS)
    colorizer = rs.colorizer()
    colorizer.set_option(rs.option.color_scheme, 8)
    align = rs.align(rs.stream.color)
    pipeline.start(config)
    print("Camera and Drone Connection Initialized.")

    try:
        set_mode("GUIDED")
        arm_vehicle()
        takeoff(
            TAKEOFF_ALTITUDE
        )  # This function will now wait until altitude is reached

        print("\n--- Phase 1: Searching for Safe Spot ---")
        while True:
            position, distance, frame = find_and_analyze_safe_spot(
                pipeline, align, safe_spot_model, colorizer
            )
            if frame is not None:
                cv2.imshow("Live Analysis", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                raise KeyboardInterrupt

            if position == "Center" and distance > 0.1:
                print(f"Target Centered! Distance: {distance:.2f} meters.")
                break
            else:
                is_clockwise = position != "Left"
                angle = (
                    SEARCH_ROTATION_ANGLE if position is None else ROTATION_ANGLE_STEP
                )
                print(
                    f"Target is {position or 'not visible'}. Rotating {'Clockwise' if is_clockwise else 'CCW'}."
                )
                rotate_yaw(angle, ROTATION_SPEED, clockwise=is_clockwise)
                # No extra sleep needed, rotate_yaw handles it

        proportional_approach(pipeline, align, safe_spot_model, colorizer)

        print("\n--- Phase 3: Landing ---")
        land_vehicle()  # This function now waits for the drone to land and disarm
        print("Mission accomplished.")

    except KeyboardInterrupt:
        print("\nMission interrupted by user. Landing immediately.")
        if master and master.motors_armed():
            land_vehicle()
    except Exception as e:
        print(f"\nAn error occurred: {e}")
        if master and master.motors_armed():
            print("Attempting to land safely.")
            land_vehicle()
    finally:
        print("Stopping camera and closing windows.")
        pipeline.stop()
        cv2.destroyAllWindows()
        if master:
            master.close()


if __name__ == "__main__":
    main()

