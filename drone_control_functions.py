# drone_control.py
# A robust, feedback-based MAVLink control library for drones.

from pymavlink import mavutil
import time
import sys
import math

# --- Global Connection Object ---
# This will be initialized by the connect_to_vehicle() function.
master = None

# --- Connection and Mode Functions ---


def connect_to_vehicle(connection_string="udpin:127.0.0.1:14550"):
    """Establishes MAVLink connection and waits for heartbeat."""
    global master
    print(f"Connecting to vehicle on: {connection_string}")
    master = mavutil.mavlink_connection(connection_string)
    master.wait_heartbeat()
    print("Heartbeat received. Vehicle connected!")
    return master


def set_mode(mode_name):
    """Sets the drone's flight mode and waits for confirmation."""
    if mode_name not in master.mode_mapping():
        sys.exit(f'Error: Unknown mode "{mode_name}".')
    mode_id = master.mode_mapping()[mode_name]
    master.mav.set_mode_send(
        master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id
    )

    # Wait for mode change to be confirmed
    while True:
        ack_msg = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=3)
        if ack_msg and ack_msg.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
            if ack_msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print(f"Switched to {mode_name} mode successfully.")
                break
            else:
                sys.exit(f"Failed to switch to {mode_name} mode.")
    time.sleep(1)


# --- Arming and State Change Functions ---


def arm_vehicle():
    """Arms the drone and waits for confirmation."""
    print("Arming vehicle...")
    master.arducopter_arm()
    master.motors_armed_wait()
    print("Vehicle Armed!")


def disarm_vehicle():
    """Disarms the drone and waits for confirmation."""
    print("Disarming vehicle...")
    master.arducopter_disarm()
    master.motors_disarmed_wait()
    print("Vehicle Disarmed!")


def land_vehicle():
    """Commands the drone to land and waits until it has disarmed."""
    print("Switching to LAND mode...")
    set_mode("LAND")
    print("Waiting for drone to land and disarm...")
    master.motors_disarmed_wait()  # This blocks until landing is complete
    print("Landed and disarmed.")


# --- Movement and Command Functions ---


def takeoff(altitude):
    """Commands takeoff and waits until the target altitude is reached."""
    print(f"Taking off to {altitude} meters...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        altitude,
    )

    # Wait until the drone reaches at least 95% of the target altitude
    while True:
        msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5)
        if msg:
            current_altitude = msg.relative_alt / 1000.0  # Altitude in meters
            print(f"  ... current altitude: {current_altitude:.2f}m")
            if current_altitude >= altitude * 0.95:
                print("Target altitude reached.")
                break
        else:
            print(
                "Warning: No altitude data received. Assuming takeoff complete after delay."
            )
            time.sleep(altitude * 2)  # Fallback sleep
            break
        time.sleep(0.5)


def rotate_yaw(angle_degrees, speed_deg_per_sec, clockwise=True):
    """Rotates drone and waits for the rotation to complete."""
    print(f"Rotating yaw by {angle_degrees} degrees...")
    direction = 1 if clockwise else -1

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        angle_degrees,  # Target angle in degrees
        speed_deg_per_sec,  # Speed
        direction,  # -1 for CCW, 1 for CW
        1,  # 1 for relative angle, 0 for absolute
        0,
        0,
        0,
    )

    # Wait for the command to complete. A simple sleep is still the most
    # practical method here without complex attitude tracking.
    # We add a small buffer.
    wait_time = (angle_degrees / speed_deg_per_sec) + 1.5
    print(f"Waiting {wait_time:.1f}s for rotation to complete...")
    time.sleep(wait_time)
    print("Rotation complete.")


def send_body_ned_velocity(vx, vy, vz):
    """
    Sends a velocity command to the drone in the BODY_OFFSET_NED frame.
    This is the core function for the proportional controller.
    """
    msg = master.mav.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # Frame relative to drone's body
        0b0000111111000111,  # Type mask (only speeds enabled)
        0,
        0,
        0,  # x, y, z positions (not used)
        vx,
        vy,
        vz,  # x, y, z velocity in m/s
        0,
        0,
        0,  # x, y, z acceleration (not used)
        0,
        0,
    )  # yaw, yaw_rate (not used)
    master.mav.send(msg)


# The main block is useful for testing individual functions
if __name__ == "__main__":
    try:
        connect_to_vehicle()
        set_mode("GUIDED")
        arm_vehicle()
        takeoff(1.5)

        print("Testing rotation...")
        rotate_yaw(90, 20, clockwise=True)
        time.sleep(2)
        rotate_yaw(90, 20, clockwise=False)

        print("Testing velocity command (forward for 2s)...")
        send_body_ned_velocity(0.5, 0, 0)  # Move forward at 0.5 m/s
        time.sleep(2)
        send_body_ned_velocity(0, 0, 0)  # Stop

        land_vehicle()
        disarm_vehicle()  # Should already be disarmed by land_vehicle, but good practice

    except Exception as e:
        print(f"An error occurred: {e}")
        # In case of error, try to land safely
        if master and master.motors_armed():
            land_vehicle()
    finally:
        if master:
            master.close()
        print("Script finished.")
