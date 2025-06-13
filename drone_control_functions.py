# Import mavutil
from pymavlink import mavutil
import time
import sys
import math # For math.radians or math.pi

# --- Global Constants and Connection Setup ---
CONNECTION_STRING = 'udpin:127.0.0.1:14550'
BAUD_RATE = 57600

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
        0,                                # param1: Speed type (1 for ground speed)
        speed,                            # param2: Speed in m/s
        0,                                # param3: Throttle (-1 indicates no change)
        0, 0, 0, 0                        # param4 ~ param7: Unused
    )
    time.sleep(1)

def takeoff(altitude=1):
    """Commands the drone to take off to a specified altitude."""
    print(f"Taking Off to {altitude} meters...")
    master.mav.command_long_send(master.target_system, master.target_component,
                                 mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
                                 0, 0, 0, 0, 0, 0, altitude)
    # Wait for a sufficient time for the drone to reach takeoff altitude
    # This duration might need adjustment based on your simulation/drone's performance
    print(f"Waiting for takeoff to complete (approx. {altitude * 3} seconds)...") # Rough estimate
    time.sleep(altitude * 3) # Simple heuristic for takeoff time

def set_body_ned_velocity(vx, vy, vz, duration=0):
    """
    Send a velocity command to the vehicle in the body frame (forward, right, down).

    Args:
        vx (float): Forward velocity in m/s.
        vy (float): Rightward velocity in m/s.
        vz (float): Downward velocity in m/s.
        duration (int): How long to maintain the velocity. 0 for indefinite.
    """
    # Type mask to command velocity and ignore position
    # 0b0000111111000111 -> ignore pos, acc, yaw, yaw_rate
    # We are commanding velocity components
    type_mask = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AFX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AFY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AFZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    )

    master.mav.set_position_target_local_ned_send(
        0,       # time_boot_ms (not used)
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # Frame relative to drone's body
        type_mask,
        0, 0, 0, # x, y, z positions (ignored)
        vx, vy, vz, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (ignored)
        0, 0     # yaw, yaw_rate (ignored)
    )
    if duration > 0:
        time.sleep(duration)

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
    time.sleep(distance_x * 1/speed_mps + 1)
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

def disarm_vehicle():
    """Disarms the drone."""
    print("Disarming vehicle...")
    master.arducopter_disarm()
    master.motors_disarmed_wait()
    print('Vehicle Disarmed!')
    time.sleep(2) # Short pause after disarming

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

        time.sleep(0.5)

        set_speed(0.25)
        # Example: Move forward 1m and rotate 90 deg
        time.sleep(3)

        move_body_ned(2.4) #moving straight x degrees, 3 here
        # rotate_yaw(180, 10, clockwise=False) # Rotate 90 degrees at 10 deg/s
        # rotate_yaw(180, 10, clockwise=False) # Rotate 90 degrees at 10 deg/s

        # time.sleep(5)

        # for _ in range(4):
        #     move_body_ned(2) #moving straight x degrees, 3 here
        #     rotate_yaw(90, 10, clockwise=False) # Rotate 90 degrees at 10 deg/s

        land_vehicle()

    except Exception as e:
        print(f"An error occurred: {e}")
        sys.exit(1)

    print("Script finished.")