# Import mavutil
from pymavlink import mavutil
import time

# Create the connection
# Need to provide the serial port and baudrate
# master = mavutil.mavlink_connection("/dev/tty.usbserial-D30JKVZM", baud=57600)

master = mavutil.mavlink_connection('udpin:127.0.0.1:14550', baud=57600)


master.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" %
      (master.target_system, master.target_component))

while True:
    msg = master.recv_match(blocking=True, type='GLOBAL_POSITION_INT').to_dict()
    print(int(msg['relative_alt'])/1000)
    msg = master.recv_match(blocking=True, type='LOCAL_POSITION_NED').to_dict()
    print(f"x: {msg['x']}, y: {msg['y']}, z: {msg['z']}")
    msg = master.recv_match(blocking=True, type='BATTERY_STATUS').to_dict()
    print(f"Voltage: {msg['voltages'][0]/1000}")
    time.sleep(0.1)