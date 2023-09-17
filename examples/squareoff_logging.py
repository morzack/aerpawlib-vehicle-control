"""
squareoff_logging will drive a vehicle (of any type) along a 10m square and use
a @background task to record the position of the vehicle. This is another good
example of how to use the StateMachine runner, as well as @background tasks and
initialize_args. Additionally, this runner hold internal state used to implement:
    - Timers
    - File objects
    - an additonal level of state (current leg)

Usage:
    python -m aerpawlib --conn ... --vehicle ... --script squareoff_logging \
        --output <optional output file> --samplerate <sample rate in Hz>

State vis:

┌───────┐drone ┌──────────┐
│ start ├──────► take_off │
└───┬───┘      └─────┬────┘
    │                │
    ├────────────────┘
    │
┌───▼───────┐
│ leg_north ├───────────┐
│           │           │
│ leg_west  │           │
│           │       ┌───▼─────────┐
│ leg_south ◄───────┤ at_position │
│           │pick   └───┬──┬────▲─┘
│ leg_east  │based on   │  └────┘sleep 5s
└───────────┘current_leg│
                    ┌───▼────┐drone ┌──────┐
                    │ finish ├──────► land │
                    └────────┘      └──────┘
"""

from argparse import ArgumentParser
import asyncio
import csv
import datetime
import time
from typing import List, TextIO
import os

from aerpawlib.runner import StateMachine, state, background, timed_state
from aerpawlib.util import VectorNED
from aerpawlib.vehicle import Drone, Rover, Vehicle

FLIGHT_ALT = 5  # m
SQUARE_SIZE = 10  # m
LOCATION_TOLERANCE = 2  # m -- ~2 is safe in general, use 3 for the rover in SITL
WAIT_TIME = 5  # s
LEG_VELOCITY = 5  # m/s


def _dump_to_csv(vehicle: Vehicle, line_num: int, writer):
    pos = vehicle.position
    lat, lon, alt = pos.lat, pos.lon, pos.alt
    volt = vehicle.battery.voltage
    timestamp = datetime.datetime.now()
    gps = vehicle.gps
    fix, num_sat = gps.fix_type, gps.satellites_visible
    if fix < 2:
        lat, lon, alt = -999, -999, -999
    vel = vehicle.velocity
    attitude = vehicle.attitude
    attitude_str = (
        "(" + ",".join(map(str, [attitude.pitch, attitude.yaw, attitude.roll])) + ")"
    )
    writer.writerow(
        [line_num, lon, lat, alt, attitude_str, vel, volt, timestamp, fix, num_sat]
    )


class SquareOff(StateMachine):
    _next_sample: float = 0
    _sampling_delay: float
    _cur_line: int
    _csv_writer: object
    _log_file: TextIO

    def initialize_args(self, extra_args: List[str]):
        # initialize extra arguments as well as any additional variables used by
        # this StateMachine
        default_file = (
            f"GPS_DATA_{datetime.datetime.now().strftime('%Y-%m-%d_%H:%M:%S')}.csv"
        )

        parser = ArgumentParser()
        parser.add_argument(
            "--output", help="log output file", required=False, default=default_file
        )
        parser.add_argument(
            "--samplerate", help="log sampling rate (Hz)", required=False, default=1, type=float
        )
        args = parser.parse_args(args=extra_args)

        self._sampling_delay = 1 / args.samplerate
        self._log_file = open(args.output, "w+")
        self._cur_line = sum(1 for _ in self._log_file) + 1
        self._csv_writer = csv.writer(self._log_file)

    @background
    async def periodic_dump(self, vehicle: Vehicle):
        # background task that (using a timer) periodically dumps vehicle status
        # into a provided file
        self._next_sample = time.time() + self._sampling_delay
        _dump_to_csv(vehicle, self._cur_line, self._csv_writer)
        self._log_file.flush()
        os.fsync(self._log_file)
        self._cur_line += 1
        await asyncio.sleep(self._sampling_delay)

    def cleanup(self):
        # called by the runner at the end of execution to clean up lingering
        # file objects/related
        self._log_file.close()

    _legs = ["leg_north", "leg_west", "leg_south", "leg_east"]
    _current_leg = 0

    @state(name="start", first=True)
    async def start(self, vehicle: Vehicle):
        # determine the type of the vehicle and branch execution based on it to
        # allow for more generic scripts
        if isinstance(vehicle, Drone):
            return "take_off"
        elif isinstance(vehicle, Rover):
            return self._legs[self._current_leg]
        raise Exception("Vehicle not supported")

    @state(name="take_off")
    async def take_off(self, drone: Drone):
        # only reachable by drones; take off and wait for it to reach
        # a specified alt
        print("taking off")
        await drone.takeoff(FLIGHT_ALT)
        print("taken off")
        await drone.set_groundspeed(LEG_VELOCITY)
        return "leg_north"

    @timed_state(name="at_position", duration=WAIT_TIME)
    async def at_position(self, _):
        # advance to the next leg, if there is a next leg
        self._current_leg += 1
        if self._current_leg < len(self._legs):
            return self._legs[self._current_leg]

        # if there are no more legs, complete the script
        return "finish"

    async def command_leg(self, vehicle: Vehicle, dNorth: float, dEast: float):
        # helper function to send a drone or rover to a specific position
        await vehicle.goto_coordinates(
            vehicle.position + VectorNED(dNorth, dEast), tolerance=LOCATION_TOLERANCE
        )

    @state(name="leg_north")
    async def leg_north(self, vehicle: Vehicle):
        print("heading north")
        await self.command_leg(vehicle, SQUARE_SIZE, 0)
        return "at_position"

    @state(name="leg_west")
    async def leg_west(self, vehicle: Vehicle):
        print("heading west")
        await self.command_leg(vehicle, 0, -SQUARE_SIZE)
        return "at_position"

    @state(name="leg_south")
    async def leg_south(self, vehicle: Vehicle):
        print("heading south")
        await self.command_leg(vehicle, -SQUARE_SIZE, 0)
        return "at_position"

    @state(name="leg_east")
    async def leg_east(self, vehicle: Vehicle):
        print("heading east")
        await self.command_leg(vehicle, 0, SQUARE_SIZE)
        return "at_position"

    @state(name="finish")
    async def finish(self, vehicle: Vehicle):
        # when done with the script, execute a special state depending on
        # vehicle type.
        if isinstance(vehicle, Drone):
            # land drones
            return "land"
        elif isinstance(vehicle, Rover):
            # rovers are done without anything special
            print("done!")
            # remember that returning nothing == script over

    @state(name="land")
    async def land(self, drone: Drone):
        await drone.land()
        print("done!")
