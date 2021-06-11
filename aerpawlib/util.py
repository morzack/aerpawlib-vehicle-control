import json
import math
from typing import List, Tuple
import dronekit

class VectorNED:
    """
    Representation of a difference between two coordinates (used for expressing
    relative motion). Makes use of NED (north, east, down) scheme.

    Units are expressed in meters

    NOTE: Something similar to this may or may not be adopted into dronkit at
    some point in the future. There are plans on their GH page, at least.
    """

    def __init__(self, north: float, east: float, down: float=0):
        self.north = north
        self.east = east
        self.down = down

class Coordinate:
    """
    An absolute point in space making use of lat, lon, and an altitude (over
    the home location of the vehicle). This is more or less equivalent to
    dronekit.LocationGlobalRelative, with more functionality added to support
    Vector3s.

    Reference for most of these functions can be found in the dronekit source
    or under some of their more specific examples. Searching for the constants
    is a pretty good way of finding them.

    Units are expressed in whatever lat/lon uses as well as meters for alt
    """

    def __init__(self, lat: float, lon: float, alt: float=0):
        self.lat = lat
        self.lon = lon
        self.alt = alt

    def location(self) -> dronekit.LocationGlobalRelative:
        return dronekit.LocationGlobalRelative(self.lat, self.lon, self.alt)

    def ground_distance(self, other) -> float:
        if not isinstance(other, Coordinate):
            raise TypeError()
        
        other = Coordinate(other.lat, other.lon, self.alt)
        return self.distance(other)

    def distance(self, other) -> float:
        if not isinstance(other, Coordinate):
            raise TypeError()
        
        # calculation uses haversine distance
        # not the most efficient approx, but as accurate as we need
        d2r = math.pi / 180
        dlon = (other.lon - self.lon) * d2r
        dlat = (other.lat - self.lat) * d2r
        a = math.pow(math.sin(dlat / 2), 2) + math.cos(self.lat*d2r) * math.cos(other.lat*d2r) * math.pow(math.sin(dlon/2), 2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        d = 6367 * c
        return math.hypot(d * 1000, other.alt - self.alt)

    def bearing(self, other) -> float:
        if not isinstance(other, Coordinate):
            raise TypeError()

        d_lat = other.lat - self.lat
        d_lon = other.lon - self.lon
        bearing = 90 + math.atan2(-d_lat, d_lon) * 57.2957795
        return bearing % 360

    def __add__(self, o):
        north = 0
        east = 0
        alt = 0
        if isinstance(o, VectorNED):
            north = o.north
            east = o.east
            alt = -o.down
        else:
            raise TypeError()

        earth_radius = 6378137.0
        d_lat = north/earth_radius
        d_lon = east/(earth_radius*math.cos(math.pi*self.lat/180))
        new_lat = self.lat + (d_lat * 180/math.pi)
        new_lon = self.lon + (d_lon * 180/math.pi)

        return Coordinate(new_lat, new_lon, self.alt + alt)

    def __sub__(self, o):
        if isinstance(o, VectorNED):
            return self + VectorNED(-o.north, -o.east, -o.down)
        elif isinstance(o, Coordinate):
            # it is, yet again, nontrivial to calculate the distance between coords.
            # here's one from wikipedia:
            # https://en.wikipedia.org/wiki/Latitude#Length_of_a_degree_of_latitude
            # https://en.wikipedia.org/wiki/Longitude#Length_of_a_degree_of_longitude
            # although since this likely isn't used a lot, i'm using an approximation
            # https://stackoverflow.com/a/19356480

            lat_mid = (self.lat + o.lat) * math.pi / 360

            d_lat = self.lat - o.lat
            d_lon = self.lon - o.lon

            return VectorNED(d_lat * (111132.954 - 559.822 * math.cos(2 * lat_mid) + 1.175 * math.cos(4 * lat_mid)),
                    d_lon * (111132.954 * math.cos(lat_mid)),
                    o.alt - self.alt)
        else:
            raise TypeError()

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
