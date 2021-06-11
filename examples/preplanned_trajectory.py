"""
preplanned_trajectory will take in a .plan file, parse it, and make the vehicle
used (must be a drone!) follow the path provided. This is a good example of how
to use the StateMachine framework, as well as the initialize_args method

Usage:
    python -m aerpawlib --conn ... --vehicle ... --script preplanned_trajectory \
            --file <.plan file to run>

State vis:
                         ┌───────────────┐
┌──────────┐         ┌───┴───────────────▼────┐
│ take_off ├─────────► take_off_do_in_transit │
└──────────┘         └──────────┬─────────────┘
                                │
       ┌────────────────────────┴───────────────┐
       │                                        │
┌──────▼────────┐    ┌────────────┐     ┌───────┴─────┐
│ next_waypoint ├────► in_transit ├─────► at_waypoint │
└──────┬────────┘    └──┬──────▲──┘     └─────────────┘
       │                └──────┘
    ┌──▼──┐
    │ rtl │
    └─────┘
"""

from argparse import ArgumentParser
from typing import List

from aerpawlib.runner import StateMachine, state
from aerpawlib.util import Coordinate, Waypoint, read_from_plan
from aerpawlib.vehicle import Drone

class PreplannedTrajectory(StateMachine):
    _waypoints: List[Waypoint]
    _current_waypoint: int=0

    def initialize_args(self, extra_args: List[str]):
        # use an extra argument parser to read in custom script arguments
        parser = ArgumentParser()
        parser.add_argument("--file", help="Mission plan file path.", required=True)
        args = parser.parse_args(args=extra_args)
        self._waypoints = read_from_plan(args.file)

    @state(name="take_off", first=True)
    def take_off(self, drone: Drone):
        # take off to the alt of the first waypoint
        takeoff_alt = self._waypoints[self._current_waypoint][3]
        drone.takeoff(takeoff_alt)
        print(f"Taking off to {takeoff_alt}")
        return "take_off_do_in_transit"

    @state(name="take_off_do_in_transit")
    def take_off_do_in_transit(self, drone: Drone):
        # wait for the drone to get to the needed alt, and then point north
        if drone.done_moving():
            drone.heading = 0
            drone.await_ready_to_move()
            return "next_waypoint"
        return "take_off_do_in_transit"

    @state(name="next_waypoint")
    def next_waypoint(self, drone: Drone):
        # figure out the next waypoint to go to
        self._current_waypoint += 1
        print(f"Waypoint {self._current_waypoint}")
        waypoint = self._waypoints[self._current_waypoint]
        if waypoint[0] == 20:       # RTL encountered, finish routine
            return "rtl"
        
        # go to next waypoint
        coords = Coordinate(*waypoint[1:4])
        drone.goto_coordinates(coords)
        return "in_transit"

    @state(name="in_transit")
    def in_transit(self, drone: Drone):
        # wait for the drone to arrive at the next waypoint
        if drone.done_moving():
            return "at_waypoint"
        return "in_transit"

    @state(name="at_waypoint")
    def at_waypoint(self, _):
        # perform any extra functionality to be done at a waypoint
        return "next_waypoint"

    @state(name="rtl")
    def rtl(self, drone: Drone):
        # return to the take off location and stop the script
        # note that this blocks, we assume that any background tasks (ex:
        # collecting data) are complete
        home_coords = Coordinate(
                drone.home_coords.lat, drone.home_coords.lon, drone.position.alt)
        drone.goto_coordinates(home_coords)
        drone.await_ready_to_move()
        drone.land()
        print("done!")
