"""
circle will make a vehicle (drone is the only supported one for now) fly in a
3m radius circle centered on where it started. This is a good example of how to
use velocity control.

Usage:
    python -m aerpawlib --conn ... --vehicle ... --script circle
"""

import argparse
import asyncio
import math

from aerpawlib.runner import StateMachine, state, in_background
from aerpawlib.util import VectorNED, Coordinate
from aerpawlib.vehicle import Drone

FLIGHT_ALT = 5      # m
CIRCLE_RAD = 10     # m
CIRCLE_VEL = 1      # m/s
N_LAPS = 3

class Circle(StateMachine):
    _target_center: Coordinate
    _point_to_center: bool=False

    def initialize_args(self, extra_args):
        parser = argparse.ArgumentParser()
        parser.add_argument("--facecenter", help="continually look at center of circle", action="store_false")
        args = parser.parse_args(args=extra_args)

        self._point_to_center = not args.facecenter

    @state(name="start", first=True)
    async def start(self, drone: Drone):
        print("taking off")
        await drone.takeoff(FLIGHT_ALT)
        self._target_center = drone.position
        print("taken off")
        return "fly_to_circumference"

    @state(name="fly_to_circumference")
    async def fly_out(self, drone: Drone):
        print("flying north to the circumference")
        await drone.goto_coordinates(self._target_center + VectorNED(CIRCLE_RAD, 0))
        return "circularize"

    _lap = 0
    _previous_thetas = []
    _prev_avg_theta = None

    @state(name="circularize")
    async def circularize(self, drone: Drone):
        current_pos = drone.position
        radius_vec = current_pos - self._target_center # points out to drone

        # calculate perpendicular/tangent vector by taking cross product w/ down
        perp_vec = radius_vec.cross_product(VectorNED(0, 0, 1))

        # normalize and ignore the height
        hypot = perp_vec.hypot(True)
        target_velocity = VectorNED(
                perp_vec.north / hypot * CIRCLE_VEL,
                perp_vec.east / hypot * CIRCLE_VEL)

        # calculate distance from ideal radius for proportional correction
        radius_err = radius_vec.hypot() - CIRCLE_RAD
        rad_correct_vec = (-1 * radius_vec) * radius_err * 0.1
        target_velocity = target_velocity + rad_correct_vec

        await drone.set_velocity(target_velocity)
        await asyncio.sleep(0.1)

        theta = math.atan2(radius_vec.north, radius_vec.east)
        self._previous_thetas.append(theta)
        if len(self._previous_thetas) > 10:
            self._previous_thetas.pop(0)
        avg_theta = sum(self._previous_thetas) / len(self._previous_thetas)
        if self._prev_avg_theta == None:
            self._prev_avg_theta = avg_theta

        if self._point_to_center:
            await drone.set_heading(math.degrees(-avg_theta)-90, blocking=False)
        
        # this condition fires when going from 3.14 rad -> -3.14 rad
        if self._prev_avg_theta > 0 and avg_theta < 0:
            self._lap += 1
        self._prev_avg_theta = avg_theta

        if self._lap > N_LAPS:
            return "rtl"

        return "circularize"

    @state(name="rtl")
    async def rtl(self, drone: Drone):
        print("returning home")
        await drone.goto_coordinates(self._target_center)
        print("landing")
        await drone.land()
        print("done!")
