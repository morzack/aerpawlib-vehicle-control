# follower script

import asyncio
from argparse import ArgumentParser
import datetime
import re
import csv
from typing import List, TextIO

from aerpawlib.external import ExternalProcess
from aerpawlib.runner import ZmqStateMachine, state, background, in_background, timed_state, at_init, sleep
from aerpawlib.util import Coordinate, Waypoint, read_from_plan_complete, VectorNED
from aerpawlib.vehicle import Drone

class FollowRunner(ZmqStateMachine):
    @state(name="launch_wait", first=True)
    async def state_start(self, _):
        return "launch_wait"

    @state(name="takeoff")
    async def state_takeoff(self, drone):
        print("taking off")
        await drone.takeoff(10)
        print("taken off")
        return "fly_to_waypoint"
    
    @state(name="fly_to_waypoint")
    async def state_waypoint(self, drone):
        await drone.goto_coordinates(drone.position + VectorNED(20, 0))
        await sleep(5)
        await self.transition_runner("leader", "waypoint_ping")
        return "waypoint_wait"
    
    @state(name="waypoint_wait")
    async def state_wait_waypoint(self, _):
        return "waypoint_wait"

    @state(name="rtl")
    async def state_rtl(self, drone):
        home_coords = Coordinate(drone.home_coords.lat, drone.home_coords.lon, drone.position.alt)
        await drone.goto_coordinates(home_coords)
        await sleep(5)
        await self.transition_runner("leader", "last_ping")
        return "wait_last_ping"
    
    @state("wait_last_ping")
    async def state_wait_last_ping(self, _):
        return "wait_last_ping"

    @state("land")
    async def state_land(self, drone):
        await drone.land()
        print("done!")
        return
