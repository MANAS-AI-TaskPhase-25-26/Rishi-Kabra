points = []

with open('boundaryTest.txt', 'r') as file:
    for line in file:
        line = line.strip()
        if not line:
            continue

        parts_lat = line.split("Lat:")
        parts_lon = parts_lat[1].split("Lon:")
        lat = float(parts_lon[0].strip())
        lon = float(parts_lon[1].strip())
        points.append((lat, lon))


#--------------

import airsim
import math

ORIGIN_LAT = -35.363261
ORIGIN_LON = 149.165230

client = airsim.MultirotorClient()
client.confirmConnection()

client.simFlushPersistentMarkers()

R = 6378137.0 
ned_points = []
labels = []

for i, (lat, lon) in enumerate(points):
    d_lat = math.radians(lat - ORIGIN_LAT)
    d_lon = math.radians(lon - ORIGIN_LON)
    
    x = d_lat * R
    y = d_lon * R * math.cos(math.radians(ORIGIN_LAT))
    
    ned_points.append(airsim.Vector3r(x, y, -0.2))
    labels.append(f"P{i+1}")

client.simPlotPoints(ned_points, color_rgba=[1.0, 0.0, 0.0, 1.0], size=35.0, is_persistent=True)
client.simPlotLineStrip(ned_points, color_rgba=[0.0, 1.0, 0.0, 1.0], thickness=10.0, is_persistent=True)
client.simPlotStrings(labels, ned_points, scale=3.0, color_rgba=[1.0, 1.0, 1.0, 1.0], duration=600.0)


import collections
import collections.abc
import time
import math
import airsim
import numpy as np
from shapely.geometry import Point, Polygon
collections.MutableMapping = collections.abc.MutableMapping
collections.Mapping = collections.abc.Mapping
collections.Sequence = collections.abc.Sequence
collections.Iterable = collections.abc.Iterable
from dronekit import connect, VehicleMode, LocationGlobalRelative


TARGET_ALTITUDE = 9.0  
GRID_STEP_METERS = 3.0   
LIDAR_STRIDE = 1



def generate_survey_grid(boundary_points, step_size_meter):
    poly = Polygon(boundary_points)
    min_lat, min_lon, max_lat, max_lon = poly.bounds

    REFERENCE_LAT = points[0][0]

    lat_step = step_size_meter / 111111.0
    lon_step = step_size_meter / (111111.0 * math.cos(math.radians(REFERENCE_LAT)))

    
    grid_points = []
    lat_range = np.arange(min_lat, max_lat, lat_step)
    lon_range = np.arange(min_lon, max_lon, lon_step) 
    
    for i, lat in enumerate(lat_range):
        row = []
        for lon in lon_range:
            p = Point(lat, lon)
            if poly.contains(p):
                row.append((lat, lon))
            
        if i % 2 == 0:
            row.reverse()
        grid_points.extend(row)
    return grid_points

STEP_SIZE = 3.0 
print(f"Generating grid with {STEP_SIZE}m steps...")
flight_path = generate_survey_grid(points, STEP_SIZE)
print(f"Total valid points inside polygon: {len(flight_path)}")

LAT_TO_METERS = 111111.0
LON_TO_METERS = 111111.0 * math.cos(math.radians(ORIGIN_LAT))

ned_boundary = []
for lat, lon in points:
    x = (lat - ORIGIN_LAT) * LAT_TO_METERS
    y = (lon - ORIGIN_LON) * LON_TO_METERS
    ned_boundary.append(airsim.Vector3r(x, y, -1.0)) #
ned_boundary.append(ned_boundary[0]) 

ned_grid = []
for lat, lon in flight_path:
    x = (lat - ORIGIN_LAT) * LAT_TO_METERS
    y = (lon - ORIGIN_LON) * LON_TO_METERS
    ned_grid.append(airsim.Vector3r(x, y, -1.0)) 

client.simPlotLineStrip(ned_boundary, color_rgba=[1.0, 0.0, 0.0, 1.0], thickness=15.0, is_persistent=True)
client.simPlotPoints(ned_grid, color_rgba=[1.0, 1.0, 0.0, 1.0], size=25.0, is_persistent=True)
print("Points plotted")


def get_distance_metres(loc1, loc2):
    REFERENCE_LAT = points[0][0]
    dlat = loc2.lat - loc1.lat
    dlong = (loc2.lon - loc1.lon) * math.cos(math.radians(REFERENCE_LAT)) 
    return math.sqrt(dlat**2 + dlong**2) * 111111.0


print("Connecting on dronekit vehicle...")
vehicle = connect('127.0.0.1:14550', wait_ready=True) 

print("Connecting to AirSim...")
airsim_client = airsim.MultirotorClient()
airsim_client.confirmConnection()

def arm_and_takeoff(target_altitude):
    print("Basic pre-arm check")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialize...")
        time.sleep(1)

    print("Waiting for GPS Lock...")
    while vehicle.gps_0.fix_type < 3:
        print(f" GPS Fix Type: {vehicle.gps_0.fix_type} (Wait for 3+)")
        time.sleep(1)

    print("Arming motors...")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print(f"Taking off to {target_altitude}m!")
    vehicle.simple_takeoff(target_altitude)

    while True:
        current_alt = vehicle.location.global_relative_frame.alt
        print(f" Current Altitude: {current_alt:.2f}m")
        if current_alt >= target_altitude * 0.95:
            print("Reached target altitude!")
            break
        time.sleep(1)




results = []

try:
    flight_path = generate_survey_grid(points, GRID_STEP_METERS)
    print(f"Grid Generated: {len(flight_path)} survey waypoints")
    arm_and_takeoff(TARGET_ALTITUDE)

    for i, wp in enumerate(flight_path):
        target = LocationGlobalRelative(wp[0], wp[1], TARGET_ALTITUDE)
        vehicle.simple_goto(target)

        while get_distance_metres(vehicle.location.global_relative_frame, target) > 2.0:
            time.sleep(0.1) 
            
        print(f"[{i+1}/{len(flight_path)}] Reached waypoint. Stabilizing for 3s...")
        time.sleep(3)

        lidar_data = airsim_client.getLidarData(lidar_name="Lidar1", vehicle_name="Copter")

        if len(lidar_data.point_cloud) >= 3:
            points_arr = np.array(lidar_data.point_cloud, dtype=np.float32).reshape((-1, 3))
            drone_state = airsim_client.getMultirotorState(vehicle_name="Copter")
            drone_x = drone_state.kinematics_estimated.position.x_val
            drone_y = drone_state.kinematics_estimated.position.y_val
            drone_alt = -drone_state.kinematics_estimated.position.z_val

            valid_points_saved = 0
            for j in range(0, len(points_arr), LIDAR_STRIDE):
                px = points_arr[j, 0]    
                py = points_arr[j, 1] 
                pz = points_arr[j, 2]

                if 0.0 < pz < 15:
                    abs_x = drone_x + px
                    abs_y = drone_y + py
                    point_elev = drone_alt - pz

                    results.append((abs_x, abs_y, point_elev))
                    valid_points_saved += 1

            print(f" -> Lidar Scan: Saved {valid_points_saved} surrounding points.")
            
        else:
            print(" -> Lidar empty. Skipped.")    

    print("\nSURVEY COMPLETE! Landing...")
    vehicle.mode = VehicleMode("LAND")

except KeyboardInterrupt:
    print("\nInterrupted by user! Landing...")
    vehicle.mode = VehicleMode("LAND")
except Exception as e:
    print(f"\nError occurred: {e}")
    vehicle.mode = VehicleMode("LAND")

finally:
    vehicle.close()
    if results:
        np.savetxt("elevation_map.csv", results, delimiter=",", header="lat,lon,elevation", comments="")
        print(f"\nSUCCESS: Data saved to elevation_map.csv ({len(results)} valid points)")