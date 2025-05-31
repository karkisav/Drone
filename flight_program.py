# Import necessary functions from the modular drone control script
# IMPORTANT: Ensure your previous code (with connect_to_vehicle, arm_vehicle, etc.)
# is saved in the same directory as this file, for example, as 'drone_control_functions.py'.
# If you saved it with a different name, update the import statement below.
from drone_control_functions import (
    connect_to_vehicle,
    arm_vehicle,
    takeoff,
    set_mode,
    move_body_ned,
    rotate_yaw,
    land_vehicle,
    master # We need to import the global master object as well
)
import sys
import time

def fly_rectangle(side_length_1, side_length_2, speed, rotation_angle_degrees, yaw_rate_degrees_per_sec):
    """
    Commands the drone to fly a rectangular path.

    Args:
        side_length_1 (float): Length of the first pair of parallel sides (meters).
        side_length_2 (float): Length of the second pair of parallel sides (meters).
        speed (float): Forward movement speed (m/s).
        rotation_angle_degrees (float): Angle to rotate at corners (e.g., 90 for a right turn).
        yaw_rate_degrees_per_sec (float): Speed of rotation (degrees/sec).
    """
    print(f"\n--- Starting Rectangle Flight: {side_length_1}m x {side_length_2}m ---")

    # Calculate movement durations for each side
    duration_1 = side_length_1 / speed
    duration_2 = side_length_2 / speed

    # Fly the first side
    print(f"Flying {side_length_1}m along side 1...")
    move_body_ned(speed, 0, 0, duration_1)
    rotate_yaw(rotation_angle_degrees, yaw_rate_degrees_per_sec)

    # Fly the second side
    print(f"Flying {side_length_2}m along side 2...")
    move_body_ned(speed, 0, 0, duration_2)
    rotate_yaw(rotation_angle_degrees, yaw_rate_degrees_per_sec)

    # Fly the third side (parallel to the first)
    print(f"Flying {side_length_1}m along side 3...")
    move_body_ned(speed, 0, 0, duration_1)
    rotate_yaw(rotation_angle_degrees, yaw_rate_degrees_per_sec)

    # Fly the fourth side (parallel to the second)
    # No rotation needed after the last side before landing
    print(f"Flying {side_length_2}m along side 4...")
    move_body_ned(speed, 0, 0, duration_2)

    print("\n--- Rectangle Flight Completed ---")


def fly_square(side_length, speed, rotation_angle_degrees, yaw_rate_degrees_per_sec):
    """
    Commands the drone to fly a square path.

    Args:
        side_length (float): Length of each side of the square (meters).
        speed (float): Forward movement speed (m/s).
        rotation_angle_degrees (float): Angle to rotate at corners (e.g., 90 for a right turn).
        yaw_rate_degrees_per_sec (float): Speed of rotation (degrees/sec).
    """
    print(f"\n--- Starting Square Flight: {side_length}m x {side_length}m ---")

    segment_duration = side_length / speed

    # Perform 4 segments of the square
    for i in range(4):
        print(f"\n--- Segment {i+1} ---")
        print(f"Moving {side_length}m forward...")
        move_body_ned(speed, 0, 0, segment_duration)

        if i < 3: # Rotate after the first three segments
            print(f"Rotating {rotation_angle_degrees} degrees slowly...")
            rotate_yaw(rotation_angle_degrees, yaw_rate_degrees_per_sec)
        else:
            print("Last segment completed, preparing for landing.")

    print("\n--- Square Flight Completed ---")


# --- Main Script Execution ---
if __name__ == "__main__":
    try:
        # 1. Connect to the vehicle
        connect_to_vehicle()

        # 2. Arm the vehicle
        arm_vehicle()

        # 3. Take off to a safe altitude
        takeoff_altitude = 1 # meters
        takeoff(takeoff_altitude)

        # 4. Ensure GUIDED mode for autonomous commands
        set_mode('GUIDED')

        # --- Choose your flight path ---
        # Option A: Fly a square
        square_side_length = 2.0 # meters
        movement_speed = 0.25    # m/s
        corner_rotation_angle = 90 # degrees (clockwise)
        slow_rotation_speed = 10 # degrees per second

        fly_square(square_side_length, movement_speed, corner_rotation_angle, slow_rotation_speed)

        # Option B: Fly a rectangle (uncomment and modify if you want a rectangle instead)
        # rect_side_1 = 2.0 # meters
        # rect_side_2 = 1.0 # meters
        # fly_rectangle(rect_side_1, rect_side_2, movement_speed, corner_rotation_angle, slow_rotation_speed)

        # 5. Land the vehicle after completing the path
        #rotate_yaw(90,10)
        land_vehicle()

    except Exception as e:
        print(f"An error occurred during the flight: {e}")
        sys.exit(1)

    print("Program finished successfully.")