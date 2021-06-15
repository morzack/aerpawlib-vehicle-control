"""
preplanned_trajectory will take in a .plan file, parse it, and make the vehicle
used (must be a drone!) follow the path provided. This is a good example of how
to use the StateMachine framework, as well as the initialize_args method

Usage:
    python -m aerpawlib --conn ... --vehicle ... --script preplanned_trajectory \
            --file <.plan file to run>

State vis:

┌──────────┐
│ take_off │
└──────┬───┘
       │
       ├────────────────────────────────────────┐
       │                                        │
┌──────▼────────┐    ┌────────────┐     ┌───────┴─────┐
│ next_waypoint ├────► in_transit ├─────► at_waypoint │
└──────┬────────┘    └────────────┘     └─────────────┘
       │
    ┌──▼──┐
    │ rtl │
    └─────┘
"""

from argparse import ArgumentParser
from typing import List

from aerpawlib.runner import StateMachine, state, in_background, timed_state
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
    async def take_off(self, drone: Drone):
        # take off to the alt of the first waypoint
        takeoff_alt = self._waypoints[self._current_waypoint][3]
        print(f"Taking off to {takeoff_alt}m")
        await drone.takeoff(takeoff_alt)
        return "next_waypoint"

    @state(name="next_waypoint")
    async def next_waypoint(self, drone: Drone):
        # figure out the next waypoint to go to
        self._current_waypoint += 1
        print(f"Waypoint {self._current_waypoint}")
        waypoint = self._waypoints[self._current_waypoint]
        if waypoint[0] == 20:       # RTL encountered, finish routine
            return "rtl"
        
        # go to next waypoint
        coords = Coordinate(*waypoint[1:4])
        in_background(drone.goto_coordinates(coords))
        return "in_transit"

    @state(name="in_transit")
    async def in_transit(self, drone: Drone):
        # wait for the drone to arrive at the next waypoint and then transition
        await drone.await_ready_to_move()
        return "at_waypoint"
    
    @timed_state(name="at_waypoint", duration=3)
    async def at_waypoint(self, _):
        # perform any extra functionality to be done at a waypoint, but stay
        # there for at least 3 seconds
        return "next_waypoint"

    @state(name="rtl")
    async def rtl(self, drone: Drone):
        # return to the take off location and stop the script
        # note that this blocks, we assume that any background tasks (ex:
        # collecting data) are complete
        home_coords = Coordinate(
                drone.home_coords.lat, drone.home_coords.lon, drone.position.alt)
        await drone.goto_coordinates(home_coords)
        await drone.land()
        print("done!")
