"""
Types and functions commonly used throughout the aerpawlib framework.
"""
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

    north: float
    east: float
    down: float

    def __init__(self, north: float, east: float, down: float=0):
        self.north = north
        self.east = east
        self.down = down

    def rotate_by_angle(self, angle: float):
        """
        Transform this VectorNED and rotate it by a certain angle, provided in
        degrees.

        ex: VectorNED(1, 0, 0).rotate_by_angle(90) -> VectorNed(0, -1, 0)
        ex: VectorNED(1, 0, 0).rotate_by_angle(45) -> VectorNed(0.707, -0.707, 0)
        """
        rads = angle / 180 * math.pi

        east = self.east * math.cos(rads) - self.north * math.sin(rads)
        north = self.east * math.sin(rads) + self.north * math.cos(rads)

        return VectorNED(north, east, self.down)

    def cross_product(self, o):
        """
        find the cross product of this and the other vector (this x o)
        """
        if not isinstance(o, VectorNED):
            raise TypeError()
        return VectorNED(
                self.east * o.down + self.down * o.east,
                self.down * o.north - self.north * o.down,
                self.north * o.east - self.east * o.north
                )

    def hypot(self, ignore_down: bool=False):
        """
        find the distance of this VectorNED, optionally ignoring any changes in
        height
        """
        if ignore_down:
            return math.hypot(self.north, self.east)
        else:
            return math.hypot(self.north, self.east, self.down)

    def __add__(self, o):
        if not isinstance(o, VectorNED):
            raise TypeError()
        return VectorNED(self.north + o.north,
                self.east + o.east,
                self.down + o.down)

    def __sub__(self, o):
        if not isinstance(o, VectorNED):
            raise TypeError()
        return VectorNED(self.north - o.north,
                self.east - o.east,
                self.down - o.down)
class Coordinate:
    """
    An absolute point in space making use of lat, lon, and an altitude (over
    the home location of the vehicle). This is more or less equivalent to
    `dronekit.LocationGlobalRelative`, with more functionality added to support
    `VectorNED`s.

    Reference for the *implementations* of most of these functions can be found
    in the dronekit source or under some of their more specific examples.
    Searching for the constants is a pretty good way of finding them.

    `lat`/`lon` should be expressed in *degrees*, while `alt` is in *meters*
    (relative to the takeoff/home location)

    `Coordinate`s can be added or subtracted with `VectorNED`s using Python's
    respective operators to calculate a new `Coordinate` relative to the
    original. This is particularly useful if you want to implement relative
    movement.

    `Coordinate`s can be subtracted from each other using Python's subtraction
    operator to calculate the vector difference between them, which is returned
    as a `VectorNED`
    """

    lat: float
    lon: float
    alt: float

    def __init__(self, lat: float, lon: float, alt: float=0):
        self.lat = lat
        self.lon = lon
        self.alt = alt

    def location(self) -> dronekit.LocationGlobalRelative:
        """
        Convert the location held by this `Coordinate` to an object usable by
        `dronekit`
        """
        return dronekit.LocationGlobalRelative(self.lat, self.lon, self.alt)

    def ground_distance(self, other) -> float:
        """
        Get the ground distance (in meters) between this `Coordinate` and
        another `Coordinate`.

        Makes use of `Coordinate.distance` under the hood
        """
        if not isinstance(other, Coordinate):
            raise TypeError()

        other = Coordinate(other.lat, other.lon, self.alt)
        return self.distance(other)

    def distance(self, other) -> float:
        """
        Get the true distance (in meters) between this `Coordinate` and another
        `Coordinate`. Unline `Coordinate.ground_distance`, this function also
        takes the altitude into account.

        The implementation used here makes use of Haversine Distance -- this
        should be extremely accurate (max err < 3m) for any distances used
        within the scope of the AERPAW program. (<10km)
        """
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
        """
        Calculate the bearing (angle) between two `Coordinates`, and return it
        in degrees
        """
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

# TODO make Waypoint use Coordinates
Waypoint = Tuple[int, float, float, float, int, float] # command, x, y, z, waypoint_id, speed

_DEFAULT_WAYPOINT_SPEED = 5 # m/s

_PLAN_CMD_TAKEOFF       = 22
_PLAN_CMD_WAYPOINT      = 16
_PLAN_CMD_RTL           = 20
_PLAN_CMD_SPEED         = 178

def read_from_plan(path: str) -> List[Waypoint]:
    """
    Helper function to read a provided .plan file (passed in as `path`) into a
    list of `Waypoint`s that can then be used to run waypoint-based missions.

    An example of this function's usage can be found under
    `examples/preplanned_trajectory.py`.

    This function has really only been tested with `.plan` files generated by
    QGroundControl. `.plan` internally stores the data as a JSON object, so it
    would be trivial to roll your own generator if needed, but with no
    assertion that this helper would work.

    Use read_from_plan_complete to get a much more generic object containing
    more functionality for each waypoint (ex: speed or time to hold)
    """
    waypoints = []
    with open(path) as f:
        data = json.load(f)
    if data["fileType"] != "Plan":
        raise Exception("Wrong file type -- use a .plan file.")
    current_speed = _DEFAULT_WAYPOINT_SPEED
    for item in data["mission"]["items"]:
        command = item["command"]
        if command in [_PLAN_CMD_TAKEOFF, _PLAN_CMD_WAYPOINT, _PLAN_CMD_RTL]:
            x, y, z = item["params"][4:7]
            waypoint_id = item["doJumpId"]
            waypoints.append((command, x, y, z, waypoint_id, current_speed))
        elif command in [_PLAN_CMD_SPEED]:
            current_speed = item["params"][1]
    return waypoints

def get_location_from_waypoint(waypoint: Waypoint) -> dronekit.LocationGlobalRelative:
    """
    Helper to get coordinates in dronekit's style from a `Waypoint`

    TODO convert/deprecate
    """
    return dronekit.LocationGlobalRelative(*waypoint[1:4])

def read_from_plan_complete(path: str):
    """
    Helper to read from a .plan file and gather all fields from each waypoint

    This can then be used for more advanced .plan file based missions
    
    Returned data schema subject to change
    """
    waypoints = []
    with open(path) as f:
        data = json.load(f)
    if data["fileType"] != "Plan":
        raise Exception("Wrong file type -- use a .plan file.")
    current_speed = _DEFAULT_WAYPOINT_SPEED
    for item in data["mission"]["items"]:
        command = item["command"]
        if command in [_PLAN_CMD_SPEED]:
            current_speed = item["params"][1]
        elif command in [_PLAN_CMD_TAKEOFF, _PLAN_CMD_WAYPOINT, _PLAN_CMD_RTL]:
            x, y, z = item["params"][4:7]
            waypoint_id = item["doJumpId"]
            delay = item["params"][0]
            waypoints.append({
                "id": waypoint_id,
                "command": command,
                "pos": [x, y, z],
                "wait_for": delay,
                "speed": current_speed,
                })
    return waypoints
