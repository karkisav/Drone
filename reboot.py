"""
Example of connecting to an autopilot via serial communication using pymavlink and then rebooting if barometer readings are not appropriate
"""

# Import mavutil
from pymavlink import mavutil

# Create the connection
# Need to provide the serial port and baudrate
master = mavutil.mavlink_connection("udpin:127.0.0.1:14550", baud=57600)

print("Waiting heartbeat")
master.wait_heartbeat()
print("Found it")


msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking='True').to_dict()

relative_alt = msg['relative_alt'] 
if 0 <= relative_alt < 300:
    pass
else:
    # Restart the ArduSub board !
    master.reboot_autopilot()