# Import mavutil
from pymavlink import mavutil
import time
import sys
import math # For math.radians or math.pi

# --- Global Constants and Connection Setup ---
CONNECTION_STRING = 'udpin:127.0.0.1:14550'
BAUD_RATE = 57600
COMMAND_RESEND_INTERVAL = 0.5 # seconds - how often to resend velocity/yaw rate commands

# Type mask for velocity control (ignoring position, acceleration, and yaw/yaw_rate)
# These are the correct constant names in pymavlink
# type_mask = (
#     mavutil.mavlink.POSITION_TARGET_TYPEMASK_VELOCITY_IGNORE |
#     mavutil.mavlink.POSITION_TARGET_TYPEMASK_ACCELERATION_IGNORE |
#     mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
#     mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
# )


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
    time.sleep(3) # Short pause after arming

def set_speed(speed=0.25):
    """
    Sets the horizontal speed of the drone
    """
    master.mav.command_long_send(
        master.target_system,             # Target system ID
        master.target_component,          # Target component ID
        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,  # Command ID
        0,                                # Confirmation
        1,                                # param1: Speed type (1 for ground speed)
        speed,                             # param2: Speed in m/s
        -1,                               # param3: Throttle (-1 indicates no change)
        0, 0, 0, 0                        # param4 ~ param7: Unused
    )
    time.sleep(1)

def takeoff(altitude=2):
    """Commands the drone to take off to a specified altitude."""
    print(f"Taking Off to {altitude} meters...")
    master.mav.command_long_send(master.target_system, master.target_component,
                                 mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
                                 0, 0, 0, 0, 0, 0, altitude)
    # Wait for a sufficient time for the drone to reach takeoff altitude
    # This duration might need adjustment based on your simulation/drone's performance
    print(f"Waiting for takeoff to complete (approx. {altitude * 3} seconds)...") # Rough estimate
    time.sleep(altitude * 3) # Simple heuristic for takeoff time

def move_body_ned(distance_x, speed_mps=0.25):
        # Send the position target command
    print("//// Moving ////")
    master.mav.set_position_target_local_ned_send(
        0,  # time_boot_ms (not used)
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # Relative to current position and heading
        int(0b110111111000),                        # int to ignore velocity, yaw, and accel
        distance_x, 0, 0,  # x, y, z positions (forward, right, down)
        0, 0, 0,                # vx, vy, vz velocities (ignored)
        0, 0, 0,                # afx, afy, afz accelerations (ignored)
        0, 0                    # yaw, yaw_rate (ignored)
    )
    time.sleep(distance_x * 1/speed_mps + 2)
    print(f"moved: {distance_x}")

def rotate_yaw(angle_degrees, speed_deg_per_sec=10, clockwise=True):
    """
    Rotate drone yaw by specified angle at given speed.
    
    Args:
        angle_degrees: Angle to rotate (e.g., 90)
        speed_deg_per_sec: Rotation speed in degrees per second
        clockwise: True for clockwise, False for counter-clockwise
    """
    print("///// Rotating /////")
    direction = 1 if clockwise else -1
    
    master.mav.command_long_send(master.target_system, master.target_component,
                             mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                             0, angle_degrees, speed_deg_per_sec, int(direction), 1, 0, 0, 0)
    
    time.sleep(angle_degrees/speed_deg_per_sec + 2)
    print(f"Rotated {angle_degrees}")

def land_vehicle():
    """Commands the drone to land."""
    print("Switching to LAND mode and waiting for landing...")
    set_mode('LAND')
    time.sleep(30) # Adjust based on your drone's landing speed

# --- Main Script Execution ---
if __name__ == "__main__":
    # This block is primarily for testing the functions directly if this file is run.
    # For the rectangle/square flight, we'll use the 'rectangle-flight-program.py'
    # which imports these functions.
    try:
        connect_to_vehicle()
        arm_vehicle()
        takeoff(altitude=1)
        set_mode('GUIDED')

        time.sleep(0.3)

        set_speed(0.25)
        # Example: Move forward 1m and rotate 90 deg
        time.sleep(0.3)

        # move_body_ned(3) #moving straight x degrees, 3 here
        # rotate_yaw(90, 10, clockwise=False) # Rotate 90 degrees at 10 deg/s

        # move_body_ned(3) #moving straight x degrees, 3 here
        # rotate_yaw(90, 10, clockwise=False) # Rotate 90 degrees at 10 deg/s

        # move_body_ned(3) #moving straight x degrees, 3 here
        # rotate_yaw(90, 10, clockwise=False) # Rotate 90 degrees at 10 deg/s

        # move_body_ned(3) #moving straight x degrees, 3 here
        # rotate_yaw(90, 10, clockwise=False) # Rotate 90 degrees at 10 deg/s

        for _ in range(4):
            move_body_ned(3) #moving straight x degrees, 3 here
            rotate_yaw(90, 10, clockwise=False) # Rotate 90 degrees at 10 deg/s

        land_vehicle()

    except Exception as e:
        print(f"An error occurred: {e}")
        sys.exit(1)

    print("Script finished.")