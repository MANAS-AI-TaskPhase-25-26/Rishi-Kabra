import collections
import collections.abc
import time

collections.MutableMapping = collections.abc.MutableMapping
collections.Mapping = collections.abc.Mapping
collections.Sequence = collections.abc.Sequence
collections.Iterable = collections.abc.Iterable

from dronekit import connect, VehicleMode, LocationGlobalRelative

print("Connecting on vehicle...")
vehicle = connect('127.0.0.1:14550', wait_ready=True)

def arm_and_takeoff(target_altitude):
    while not vehicle.is_armable:
        print("Waiting for vehicle to initialize...")
        time.sleep(1)

    print("Arming motors...")
    
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)
    
    print(f"Taking off to {target_altitude}m")
    vehicle.simple_takeoff(target_altitude)

    while True:
        print(" Current Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= 0.95 * target_altitude:
            print("Reached target altitude")
            break
        time.sleep(1)


try:
    print("Vehicle attributes")
    print(vehicle.gps_0)
    print(vehicle.battery)
    print(vehicle.last_heartbeat)
    print(vehicle.is_armable)
    print(vehicle.mode.name)
    current_gps = vehicle.location.global_frame
    # alt above takeoff point
    current_rel = vehicle.location.global_relative_frame
    print(current_gps)
    print(current_rel)

    arm_and_takeoff(10)

    print("Hovering for 10 seconds")
    time.sleep(10)

    print("LANDING...")
    vehicle.mode = VehicleMode("LAND")

    while vehicle.armed:
        print("Final descent - Altitude: ", vehicle.location.global_relative_frame.alt)
        time.sleep(1)

except KeyboardInterrupt:
    print("\nScript interrupted. LANDING")
    vehicle.mode = VehicleMode("LAND")
finally:
    print("Closing vehicle connection")
    vehicle.close()