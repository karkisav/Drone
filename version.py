from pymavlink import mavutil

# Establish connection (adjust the connection string as needed)
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')

# Wait for the heartbeat message to find the system ID
master.wait_heartbeat()

# Send command to request autopilot version
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES,
    0,
    1, 0, 0, 0, 0, 0, 0
)


# Wait for the AUTOPILOT_VERSION message
msg = master.recv_match(type='AUTOPILOT_VERSION', blocking=True)

# Extract and print the firmware version
if msg:
    flight_sw_version = msg.flight_sw_version
    major = (flight_sw_version >> 24) & 0xFF
    minor = (flight_sw_version >> 16) & 0xFF
    patch = (flight_sw_version >> 8) & 0xFF
    print(f"Firmware Version: {major}.{minor}.{patch}")
else:
    print("AUTOPILOT_VERSION message not received.")
