# Import mavutil
from pymavlink import mavutil
import time
import sys

# Create the connection
master = mavutil.mavlink_connection('udpin:127.0.0.1:14550', baud=57600)
# Wait a heartbeat before sending commands
master.wait_heartbeat()

# https://mavlink.io/en/messages/common.html#MAV_CMD_COMPONENT_ARM_DISARM

# Arm
master.arducopter_arm()
# master.mav.command_long_send(
#     master.target_system,
#     master.target_component,
#     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
#     0,
#     1, 0, 0, 0, 0, 0, 0)

# wait until arming confirmed (can manually check with master.motors_armed())
print("Waiting for the vehicle to arm")
master.motors_armed_wait()
print('Armed!')

time.sleep(2)

print("Taking-Off ...")
master.mav.command_long_send(master.target_system, master.target_component,
                             mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                             0, 0, 0, 0, 0, 0, 0, 1)

time.sleep(5)

mode = 'GUIDED'
mode_id = master.mode_mapping()[mode]
master.set_mode(mode_id)
time.sleep(10)

# print("Rotating")
# master.mav.command_long_send(master.target_system, master.target_component,
#                              mavutil.mavlink.MAV_CMD_CONDITION_YAW,
#                              0, 90, 10, -1, 1, 0, 0, 0)

# master.mav.command_long_send(
#     master.target_system,             # Target system ID
#     master.target_component,          # Target component ID
#     mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # Command ID
#     0,                                # Confirmation
#     90,                               # param1: Yaw angle in degrees (positive for clockwise)
#     15,                               # param2: Yaw speed in deg/s
#     1,                                # param3: Direction (-1: CCW, 1: CW)
#     1,                                # param4: Relative (1) or absolute (0) angle
#     0, 0, 0                           # param5 ~ param7: Unused
# )

# master.mav.set_position_target_local_ned_send(
#     0,  # time_boot_ms (not used)
#     master.target_system,
#     master.target_component,
#     mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # Relative to current position and heading
#     int(0b110111111000),                        # int to ignore velocity, yaw, and accel
#     3, 0, 0,  # x, y, z positions (forward, right, down)
#     0, 0, 0,                # vx, vy, vz velocities (ignored)
#     0, 0, 0,                # afx, afy, afz accelerations (ignored)
#     0, 0                    # yaw, yaw_rate (ignored)
# )


# time.sleep(5)


mode = 'LAND'
print("Mode chaned to: LAND")

# Check if mode is available
if mode not in master.mode_mapping():
    print('Unknown mode : {}'.format(mode))
    print('Try:', list(master.mode_mapping().keys()))
    sys.exit(1)

print("Landing")
# Get mode ID
mode_id = master.mode_mapping()[mode]
# Set new mode
# master.mav.command_long_send(
#    master.target_system, master.target_component,
#    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
#    0, mode_id, 0, 0, 0, 0, 0) or:']
# master.set_mode(mode_id) or:
master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id)

