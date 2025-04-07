"""
Connecting and Takeoff
"""

# Import mavutil
from pymavlink import mavutil
import time

# Create the connection
# Need to provide the serial port and baudrate
master = mavutil.mavlink_connection("/dev/tty.usbserial-0001", baud=57600)

print("Waiting heartbeat")
master.wait_heartbeat()
print("Found it")

msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking='True').to_dict()
relative_alt = msg['relative_alt'] 
print(relative_alt)


#Arming Command
print("Arming")
master.mav.command_long_send(master.target_system, master.target_component,
                             mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                             0, 0, 0, 0, 0, 0, 0, 0)

msg = master.recv_match(type='COMMAND_ACK', blocking='True')
print(msg)

print("Taking-Off ...")
master.mav.command_long_send(master.target_system, master.target_component,
                             mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                             0, 0, 0, 0, 0, 0, 0, 1)


time.sleep(5)



msg = master.recv_match(type='COMMAND_ACK', blocking='True')
print(msg)