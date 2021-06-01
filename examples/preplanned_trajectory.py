from argparse import ArgumentParser
import json
from typing import List, Tuple

from aerpawlib.runner import StateMachine, state
from aerpawlib.vehicle import Drone
from dronekit import LocationGlobalRelative

_Waypoint = Tuple[int, float, float, float, int] # command, x, y, z, waypoint_id

def _read_mission(path: str) -> List[_Waypoint]:
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

class PreplannedTrajectory(StateMachine):
    _waypoints: List[_Waypoint]
    _current_waypoint: int=0

    def initialize_args(self, extra_args: List[str]):
        # load in things from file
        parser = ArgumentParser()
        parser.add_argument("--file", help="Mission plan file path.", required=True)
        args = parser.parse_args(args=extra_args)
        self._waypoints = _read_mission(args.file)

    @state(name="take_off", first=True)
    def take_off(self, drone: Drone):
        takeoff_alt = self._waypoints[self._current_waypoint][3]
        drone.takeoff(takeoff_alt)
        print(f"Taking off to {takeoff_alt}")
        return "take_off_do_in_transit"

    @state(name="take_off_do_in_transit")
    def take_off_do_in_transit(self, drone: Drone):
        if drone.done_moving():
            drone.heading = 0
            drone.await_ready_to_move()
            return "next_waypoint"

        # here's where doInTransit would be called as needed (periodically)
        return "take_off_do_in_transit"

    @state(name="next_waypoint")
    def next_waypoint(self, drone: Drone):
        self._current_waypoint += 1
        print(f"Waypoint f{self._current_waypoint}")
        waypoint = self._waypoints[self._current_waypoint]
        if waypoint[0] == 20:       # RTL encountered, finish routine
            return "rtl"
        
        # go to next waypoint
        coords = LocationGlobalRelative(*waypoint[1:4])
        drone.goto_coordinates(coords)
        return "in_transit"

    @state(name="in_transit")
    def in_transit(self, drone: Drone):
        if drone.done_moving():
            return "at_waypoint"
        return "in_transit"

    @state(name="at_waypoint")
    def at_waypoint(self, _):
        return "next_waypoint"

    @state(name="rtl")
    def rtl(self, drone: Drone):
        home_coords = LocationGlobalRelative(
                drone.home_coords.lat, drone.home_coords.lon, drone.position.alt)
        drone.goto_coordinates(home_coords)
        drone.await_ready_to_move()
        drone.land()
        print("done!")
