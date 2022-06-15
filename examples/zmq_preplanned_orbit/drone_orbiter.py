import asyncio
from argparse import ArgumentParser
import datetime
import re
import math
import csv
from typing import List, TextIO

from aerpawlib.external import ExternalProcess
from aerpawlib.runner import ZmqStateMachine, state, background, in_background, timed_state, at_init, sleep, expose_field_zmq
from aerpawlib.util import Coordinate, Waypoint, read_from_plan_complete, VectorNED
from aerpawlib.vehicle import Drone

from consts import *

class OrbiterRunner(ZmqStateMachine):
    @expose_field_zmq(name="position")
    async def get_drone_position(self, drone: Drone):
        return drone.position

    @state(name="wait_loop", first=True)
    async def state_wait_loop(self, _):
        # used to continually wait for an action to come in
        await asyncio.sleep(0.1)
        return "wait_loop"

    @state(name="take_off")
    async def state_take_off(self, drone: Drone):
        print("taking off")
        await drone.takeoff(TAKEOFF_ALT)
        await self.transition_runner(ZMQ_GROUND, "callback_orbiter_taken_off")
        return "wait_loop"

    @state(name="next_waypoint")
    async def state_next_waypoint(self, drone: Drone):
        coords = await self.query_field(ZMQ_GROUND, "orbiter_next_waypoint")
        await drone.goto_coordinates(coords)
        await self.transition_runner(ZMQ_GROUND, "callback_orbiter_at_waypoint")
        return "wait_loop"

    _orbit_coord_center: Coordinate
    _orbit_end_position: Coordinate

    _previous_thetas = []
    _prev_avg_theta = None
    _initial_theta = None
    
    @state(name="orbit_tracer")
    async def state_orbit_tracer_start(self, drone: Drone):
        self._orbit_coord_center = await self.query_field(ZMQ_TRACER, "position")
        self._orbit_end_position = drone.position
        self._previous_thetas = []
        self._prev_avg_theta = None
        self._initial_theta = None
        return "orbit"

    @state(name="orbit")
    async def state_orbit(self, drone: Drone):
        current_pos = drone.position
        radius_vec = current_pos - self._orbit_coord_center # points out to drone

        # calculate perpendicular/tangent vector by taking cross product w/ down
        perp_vec = radius_vec.cross_product(VectorNED(0, 0, 1))

        # normalize and ignore the height
        hypot = perp_vec.hypot(True)
        target_velocity = VectorNED(
                perp_vec.north / hypot * ORBIT_VEL,
                perp_vec.east / hypot * ORBIT_VEL)

        await drone.set_velocity(target_velocity)
        await asyncio.sleep(0.05)

        theta = math.atan2(radius_vec.north, radius_vec.east)
        if self._initial_theta == None:
            self._initial_theta = theta
        theta -= self._initial_theta
        self._previous_thetas.append(theta)
        if len(self._previous_thetas) > 10:
            self._previous_thetas.pop(0)
        avg_theta = sum(self._previous_thetas) / len(self._previous_thetas)
        if self._prev_avg_theta == None:
            self._prev_avg_theta = avg_theta
        
        # this condition fires when going from 3.14 rad -> -3.14 rad
        if self._prev_avg_theta > 0 and avg_theta < 0:
            await self.transition_runner(ZMQ_GROUND, "callback_orbiter_orbit_done")
            return "wait_loop"
        
        return "orbit"
    
    @state(name="rtl")
    async def state_rtl(self, drone: Drone):
        home_coords = Coordinate(
                drone.home_coords.lat, drone.home_coords.lon, drone.position.alt)
        await drone.goto_coordinates(home_coords)
        await self.transition_runner(ZMQ_GROUND, "callback_orbiter_rtl_done")
        return "wait_loop"

    @state(name="land")
    async def state_land(self, drone: Drone):
        await drone.land()
        return
