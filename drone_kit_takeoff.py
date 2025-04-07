#!/usr/bin/env python
# -*- coding: utf-8 -*-

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
import time
import math

# Connect to the Vehicle
print("Connecting to vehicle...")
# Replace with your connection string (e.g., '/dev/ttyAMA0', '/dev/ttyACM0', etc.)
vehicle = connect('/dev/tty.usbserial-0001', baud=57600)

# Define arm and takeoff function
def arm_and_takeoff(target_altitude):
    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print("Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(target_altitude)

    # Wait until the vehicle reaches the target altitude
    while True:
        current_altitude = vehicle.rangefinder.distance or vehicle.location.global_relative_frame.alt
        print("Altitude: %f" % current_altitude)
        
        if current_altitude >= target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

# Main mission
try:
    # Arm and takeoff to 3 meters
    arm_and_takeoff(2)
    
    print("Holding position for 30 seconds...")
    time.sleep(30)
    
    print("Landing...")
    vehicle.mode = VehicleMode("LAND")
    
    # Wait for landing to complete
    while vehicle.armed:
        print("Waiting for landing and disarm...")
        time.sleep(2)
    
    print("Mission complete!")

finally:
    # Close vehicle object
    vehicle.close()