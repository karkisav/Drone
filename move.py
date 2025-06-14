# Import mavutil
from pymavlink import mavutil
import time
import sys
import math # For math.radians or math.pi

# --- Global Constants and Connection Setup ---
CONNECTION_STRING = 'udpin:127.0.0.1:14550'
BAUD_RATE = 57600

# Type mask for velocity control (ignoring position, acceleration, and yaw/yaw_rate)
# This explicit bitmask tells the autopilot to only consider vx, vy, vz.
VELOCITY_CONTROL_MASK = (
    mavutil.mavlink.MAV_POS_TARGET_TYPE_MASK_X_IGNORE |
    mavutil.mavlink.MAV_POS_TARGET_TYPE_MASK_Y_IGNORE |
    mavutil.mavlink.MAV_POS_TARGET_TYPE_MASK_Z_IGNORE |
    mavutil.mavlink.MAV_POS_TARGET_TYPE_MASK_AX_IGNORE |
    mavutil.mavlink.MAV_POS_TARGET_TYPE_MASK_AY_IGNORE |
    mavutil.mavlink.MAV_POS_TARGET_TYPE_MASK_AZ_IGNORE |
    mavutil.mavlink.MAV_POS_TARGET_TYPE_MASK_YAW_IGNORE |
    mavutil.mavlink.MAV_POS_TARGET_TYPE_MASK_YAW_RATE_IGNORE
)

# Type mask for yaw rate control (ignoring position, velocity, and acceleration)
YAW_RATE_CONTROL_MASK = (
    mavutil.mavlink.MAV_POS_TARGET_TYPE_MASK_X_IGNORE |
    mavutil.mavlink.MAV_POS_TARGET_TYPE_MASK_Y_IGNORE |
    mavutil.mavlink.MAV_POS_TARGET_TYPE_MASK_Z_IGNORE |
    mavutil.mavlink.MAV_POS_TARGET_TYPE_MASK_VX_IGNORE |
    mavutil.mavlink.MAV_POS_TARGET_TYPE_MASK_VY_IGNORE |
    mavutil.mavlink.MAV_POS_TARGET_TYPE_MASK_VZ_IGNORE |
    mavutil.mavlink.MAV_POS_TARGET_TYPE_MASK_AX_IGNORE |
    mavutil.mavlink.MAV_POS_TARGET_TYPE_MASK_AY_IGNORE |
    mavutil.mavlink.MAV_POS_TARGET_TYPE_MASK_AZ_IGNORE |
    mavutil.mavlink.MAV_POS_TARGET_TYPE_MASK_YAW_IGNORE # We are controlling yaw_rate, so yaw is ignored
)

# Global master connection object (will be initialized in main)
master = None

# --- Helper Functions ---

def connect_to_vehicle():
    """Establishes MAVLink connection and waits for heartbeat."""
    global master
    print(f"Connecting to vehicle at {CONNECTION_STRING}...")
    master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE)
    master.wait_heartbeat()
    print("Heartbeat received. Vehicle connected!")

def set_mode(mode_name):
    """Sets the drone's flight mode."""
    if mode_name not in master.mode_mapping():
        sys.exit(f'Error: Unknown mode "{mode_name}". Available modes: {list(master.mode_mapping().keys())}')
    mode_id = master.mode_mapping()[mode_name]
    master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)
    print(f"Switched to {mode_name} mode.")
    time.sleep(1) # Give a moment for mode change to register

def arm_vehicle():
    """Arms the drone and waits for confirmation."""
    print("Arming vehicle...")
    master.arducopter_arm()
    master.motors_armed_wait()
    print('Vehicle Armed!')
    time.sleep(1) # Short pause after arming

def takeoff(altitude):
    """Commands the drone to take off to a specified altitude."""
    print(f"Taking Off to {altitude} meters...")
    master.mav.command_long_send(master.target_system, master.target_component,
                                 mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
                                 0, 0, 0, 0, 0, 0, altitude)
    # Wait for a sufficient time for the drone to reach takeoff altitude
    # This duration might need adjustment based on your simulation/drone's performance
    print(f"Waiting for takeoff to complete (approx. {altitude * 3} seconds)...") # Rough estimate
    time.sleep(altitude * 3) # Simple heuristic for takeoff time

def move_body_ned(vx, vy, vz, duration):
    """
    Commands the drone to move with specified velocities in its body frame.
    vx: forward/backward velocity (m/s)
    vy: right/left velocity (m/s)
    vz: down/up velocity (m/s) (positive is down)
    duration: how long to apply the velocity (seconds)
    """
    print(f"Moving at vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f} m/s for {duration:.2f} seconds...")
    master.mav.set_position_target_local_ned_send(
        0, # time_boot_ms (not used)
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED, # Frame: Relative to the drone's body
        VELOCITY_CONTROL_MASK,
        0, 0, 0, # x, y, z position (ignored by mask)
        vx, vy, vz, # vx, vy, vz velocity
        0, 0, 0, # afx, afy, afz acceleration (ignored by mask)
        0, 0 # yaw, yaw_rate (ignored by mask)
    )
    time.sleep(duration)
    # Stop movement after duration
    print("Stopping movement...")
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        VELOCITY_CONTROL_MASK,
        0, 0, 0, # Position (ignored)
        0, 0, 0, # vx, vy, vz velocity (zero to stop)
        0, 0, 0, # Acceleration (ignored)
        0, 0 # Yaw, Yaw Rate (ignored)
    )
    time.sleep(0.5) # Short pause to ensure stop command is processed

def rotate_yaw(target_angle_degrees, yaw_rate_degrees_per_sec=10):
    """
    Rotates the drone by a specified angle at a given yaw rate.
    target_angle_degrees: The total angle to rotate (e.g., 90 for 90 degrees clockwise).
    yaw_rate_degrees_per_sec: How fast to rotate in degrees per second.
    """
    target_angle_radians = math.radians(target_angle_degrees)
    yaw_rate_radians_per_sec = math.radians(yaw_rate_degrees_per_sec)

    if yaw_rate_radians_per_sec == 0:
        print("Yaw rate is zero, no rotation will occur.")
        return

    duration = abs(target_angle_radians / yaw_rate_radians_per_sec)
    actual_yaw_rate = yaw_rate_radians_per_sec if target_angle_degrees >= 0 else -yaw_rate_radians_per_sec

    print(f"Rotating {target_angle_degrees} degrees at {yaw_rate_degrees_per_sec} deg/s for {duration:.2f} seconds...")

    master.mav.set_position_target_local_ned_send(
        0, # time_boot_ms (not used)
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED, # Frame: Relative to the drone's body
        YAW_RATE_CONTROL_MASK,
        0, 0, 0, # x, y, z position (ignored)
        0, 0, 0, # vx, vy, vz velocity (ignored)
        0, 0, 0, # afx, afy, afz acceleration (ignored)
        0, actual_yaw_rate # yaw (ignored), yaw_rate (controlled)
    )
    time.sleep(duration)

    # Stop rotation
    print("Stopping rotation...")
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        YAW_RATE_CONTROL_MASK,
        0, 0, 0, # Position (ignored)
        0, 0, 0, # vx, vy, vz velocity (ignored)
        0, 0, 0, # Acceleration (ignored)
        0, 0 # yaw (ignored), yaw_rate (zero to stop)
    )
    time.sleep(0.5) # Short pause to ensure stop command is processed

def land_vehicle():
    """Commands the drone to land."""
    print("Switching to LAND mode and waiting for landing...")
    set_mode('LAND')
    time.sleep(30) # Adjust based on your drone's landing speed

# --- Main Script Execution ---
if __name__ == "__main__":
    try:
        connect_to_vehicle()
        arm_vehicle()
        takeoff(altitude=3) # Take off to 3 meters

        set_mode('GUIDED') # Ensure GUIDED mode for movement commands

        # Define movement parameters for the square
        segment_distance = 1.0  # meters
        segment_speed = 0.25    # m/s
        segment_duration = segment_distance / segment_speed

        rotation_angle = 90     # degrees
        slow_yaw_rate = 15      # degrees per second (adjust for desired slowness)

        # Perform 3 segments of the square
        for i in range(3):
            print(f"\n--- Segment {i+1} ---")
            print(f"Moving {segment_distance}m forward...")
            move_body_ned(segment_speed, 0, 0, segment_duration)

            if i < 2: # No need to rotate after the last segment before landing
                print(f"Rotating {rotation_angle} degrees slowly...")
                rotate_yaw(rotation_angle, slow_yaw_rate)
            else:
                print("Last segment completed, preparing for landing.")

        # After the square, land the vehicle
        land_vehicle()

    except Exception as e:
        print(f"An error occurred: {e}")
        sys.exit(1)

    print("Script finished.")
