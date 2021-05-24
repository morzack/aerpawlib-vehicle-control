import json
import math
from typing import List, Tuple
import dronekit

def calc_location_delta(location: dronekit.LocationGlobalRelative, dNorth: float, dEast: float) -> dronekit.LocationGlobalRelative:
    """
    Calculate a location relative to an existing set of coordninates given a
    set of offsets (in meters)

    This method is fairly accurate over small distances, except when close to
    the poles
    
    Borrowed from DroneKit source/examples

    For more info, see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    lat, lon = location.lat, location.lon

    earth_radius = 6378137.0
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*lat/180))

    #New position in decimal degrees
    newlat = lat + (dLat * 180/math.pi)
    newlon = lon + (dLon * 180/math.pi)

    return dronekit.LocationGlobalRelative(newlat, newlon, location.alt)

def calc_distance(location_a: dronekit.LocationGlobalRelative, location_b: dronekit.LocationGlobalRelative) -> float:
    """
    Calculate the ground distance between two coordinates, in meters.

    This uses an approximation found in DroneKit source and other repos online
    """
    dLat = location_b.lat - location_a.lat
    dLon = location_b.lon - location_a.lon
    return math.hypot(dLat, dLon) * 1.113195e5

def calc_bearing(location_a: dronekit.LocationGlobalRelative, location_b: dronekit.LocationGlobalRelative) -> float:
    """
    Calculate the bearing between two coordinates, in degrees
    
    This uses an approximation found in DroneKit source and other repos online
    """
    off_x = location_b.lon - location_a.lon
    off_y = location_b.lat - location_a.lat
    bearing = 90 + math.atan2(-off_y, off_x) * 57.2957795
    return bearing % 360

Waypoint = Tuple[int, float, float, float, int] # command, x, y, z, waypoint_id

def read_from_plan(path: str) -> List[Waypoint]:
    """
    Read a provided .plan file (passed in as <path>) into a list of Waypoints
    that can then be used to run waypoint-based missions
    """
    waypoints = []
    with open(path) as f:
        data = json.load(f)
    if data["fileType"] != "Plan":
        raise Exception("Wrong file type -- use a .plan file.")
    for item in data["mission"]["items"]:
        command = item["command"]
        x, y, z = item["params"][4:7]
        waypoint_id = item["doJumpId"]
        waypoints.append((command, x, y, z, waypoint_id))
    x_home, y_home = data["mission"]["plannedHomePosition"][0:2]
    z_home = waypoints[0][3]
    waypoints.append((16, x_home, y_home, z_home, len(waypoints)+1))
    return waypoints

def get_location_from_waypoint(waypoint: Waypoint) -> dronekit.LocationGlobalRelative:
    """
    Helper to get coordinates usable in most methods from a Waypoint
    """
    return dronekit.LocationGlobalRelative(*waypoint[1:4])
