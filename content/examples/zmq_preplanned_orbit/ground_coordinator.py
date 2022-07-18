"""
zmq_preplanend_orbit mission

there are two drones and a ground station

one drone, the tracer, follows a preplanned trajectory file
the other drone, the orbiter, will orbit the tracer at each waypoint

the ground coordinator will read from the file and issue controls to each drone (ex: go to this waypoint or orbit this waypoint)

the pattern proposed in this script is to have each "command" be a state on individual drones
thus, to make a drone do something, we transition their state from the central controller
"""

import asyncio
from argparse import ArgumentParser
import datetime
import re
import csv
from typing import List, TextIO

from aerpawlib.external import ExternalProcess
from aerpawlib.runner import ZmqStateMachine, state, background, in_background, timed_state, at_init, sleep, expose_field_zmq
from aerpawlib.util import Coordinate, Waypoint, read_from_plan_complete
from aerpawlib.vehicle import Drone

from consts import *

class GroundCoordinatorRunner(ZmqStateMachine):
    _waypoints = []
    _current_waypoint: int=0

    def initialize_args(self, extra_args: List[str]):
        # use an extra argument parser to read in custom script arguments
        parser = ArgumentParser()
        parser.add_argument("--file", help="Mission plan file path.", required=True)
        args = parser.parse_args(args=extra_args)
        self._waypoints = read_from_plan_complete(args.file)

    @state(name="take_off", first=True)
    async def state_take_off(self, _):
        # make both drones take off at the same time
        print("taking off")
        await self.transition_runner(ZMQ_TRACER, "take_off"),
        await self.transition_runner(ZMQ_ORBITER, "take_off"),
        print("taking off sent")
        return "await_taken_off"

    _tracer_taken_off = False
    _orbiter_taken_off = False

    @state(name="await_taken_off")
    async def state_await_taken_off(self, _):
        # wait for both drones to finish taking off
        # this will be done by waiting for two flags to be set; each flag is set by transitioning to a special state
        if not (self._tracer_taken_off and self._orbiter_taken_off):
            return "await_taken_off"
        return "next_waypoint"

    @state(name="callback_tracer_taken_off")
    async def callback_tracer_taken_off(self, _):
        self._tracer_taken_off = True
        return "await_taken_off"
    
    @state(name="callback_orbiter_taken_off")
    async def callback_orbiter_taken_off(self, _):
        self._orbiter_taken_off = True
        return "await_taken_off"


    _tracer_at_waypoint = False
    _orbiter_at_waypoint = False
    
    @state(name="next_waypoint")
    async def state_next_waypoint(self, _):
        # send the tracer to a specific point, make the orbiter follow the same *vector* (i.e. they take two parallel paths)
        self._current_waypoint += 1
        if self._current_waypoint >= len(self._waypoints):
            return "rtl"
        print(f"Waypoint {self._current_waypoint}")
        
        waypoint = self._waypoints[self._current_waypoint]
        if waypoint["command"] == 20:       # RTL encountered, finish routine
            return "rtl"
        
        await self.transition_runner(ZMQ_TRACER, "next_waypoint"),
        await self.transition_runner(ZMQ_ORBITER, "next_waypoint"),

        self._tracer_at_waypoint = False
        self._orbiter_at_waypoint = False
        
        return "await_in_transit"

    # funcs to calculate where each drone should go
    @expose_field_zmq(name="tracer_next_waypoint")
    async def get_tracer_next_waypoint(self, _):
        waypoint = self._waypoints[self._current_waypoint]
        coords = Coordinate(*waypoint["pos"])
        return coords
    
    @expose_field_zmq(name="orbiter_next_waypoint")
    async def get_orbiter_next_waypoint(self, _):
        # orbiter travels in a path parallel to tracer
        waypoint = self._waypoints[self._current_waypoint]
        coords = Coordinate(*waypoint["pos"])
        tracer_pos = await self.query_field(ZMQ_TRACER, "position")
        tracer_delta = coords - tracer_pos
        orbiter_pos = await self.query_field(ZMQ_ORBITER, "position")
        orbiter_next_pos = orbiter_pos + tracer_delta
        return orbiter_next_pos
    
    @state(name="await_in_transit")
    async def state_await_in_transit(self, _):
        # wait for both drones to finish moving
        # this will be done by waiting for two flags to be set; each flag is set by transitioning to a special state
        if not (self._tracer_at_waypoint and self._orbiter_at_waypoint):
            return "await_in_transit"
        return "orbiter_start_orbit"

    @state(name="callback_tracer_at_waypoint")
    async def callback_tracer_at_waypoint(self, _):
        self._tracer_at_waypoint = True
        return "await_in_transit"
    
    @state(name="callback_orbiter_at_waypoint")
    async def callback_orbiter_at_waypoint(self, _):
        self._orbiter_at_waypoint = True
        return "await_in_transit"
    
    
    _orbiter_orbit_done = False

    @state(name="orbiter_start_orbit")
    async def state_orbiter_start_orbit(self, _):
        self._orbiter_orbit_done = False
        await self.transition_runner(ZMQ_ORBITER, "orbit_tracer")
        return "await_orbiter_orbiting"

    @state(name="await_orbiter_orbiting")
    async def state_await_orbiter_oribiting(self, _):
        if self._orbiter_orbit_done:
            return "next_waypoint"
        return "await_orbiter_orbiting"

    @state(name="callback_orbiter_orbit_done")
    async def callback_orbiter_orbit_done(self, _):
        self._orbiter_orbit_done = True
        return "await_orbiter_orbiting"
    

    _tracer_rtl_done = False
    _orbiter_rtl_done = False

    @state(name="rtl")
    async def state_rtl(self, _):
        await self.transition_runner(ZMQ_TRACER, "rtl"),
        await self.transition_runner(ZMQ_ORBITER, "rtl"),
        self._tracer_rtl_done = False
        self._orbiter_rtl_done = False
        return "await_rtl_done"
    
    @state(name="await_rtl_done")
    async def state_await_rtl_done(self, _):
        if not (self._tracer_rtl_done and self._orbiter_rtl_done):
            return "await_rtl_done"
        return "land"
    
    @state(name="callback_tracer_rtl_done")
    async def callback_tracer_rtl_done(self, _):
        self._tracer_rtl_done = True
        return "await_rtl_done"
    
    @state(name="callback_orbiter_rtl_done")
    async def callback_orbiter_rtl_done(self, _):
        self._orbiter_rtl_done = True
        return "await_rtl_done"

    @state(name="land")
    async def state_land(self, _):
        await self.transition_runner(ZMQ_TRACER, "land"),
        await self.transition_runner(ZMQ_ORBITER, "land"),
        await asyncio.sleep(5)
        return
