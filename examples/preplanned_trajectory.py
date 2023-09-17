"""
preplanned_trajectory will take in a .plan file, parse it, and make the vehicle
used (must be a drone!) follow the path provided. This is a good example of how
to use the StateMachine framework, as well as the initialize_args method. This
script also contains an example of how to use the ExternalProcess tooling, in
this case by calling ping periodically (at every waypoint)

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

import asyncio
from argparse import ArgumentParser
import datetime
import re
import csv
import os
from typing import List, TextIO

from aerpawlib.external import ExternalProcess
from aerpawlib.runner import (
    StateMachine,
    state,
    background,
    in_background,
    timed_state,
    at_init,
    sleep,
)
from aerpawlib.util import Coordinate, Waypoint, read_from_plan_complete
from aerpawlib.vehicle import Drone, Rover, Vehicle


class PreplannedTrajectory(StateMachine):
    _waypoints = []
    _waypoint_fname: str
    _current_waypoint: int = 0

    _default_leg_speed: float = None
    _default_heading: float = None

    _next_sample: float = 0
    _sampling_delay: float
    _cur_line: int
    _csv_writer: object
    _log_file: TextIO

    def initialize_args(self, extra_args: List[str]):
        # use an extra argument parser to read in custom script arguments
        default_file = (
            f"GPS_DATA_{datetime.datetime.now().strftime('%Y-%m-%d_%H:%M:%S')}.csv"
        )

        parser = ArgumentParser()
        parser.add_argument("--file", help="Mission plan file path.", required=True)
        parser.add_argument("--ping", help="call ping coroutine", action="store_false")
        parser.add_argument(
            "--skipoutput", help="don't dump gps data to a file", action="store_false"
        )
        parser.add_argument(
            "--output", help="log output file", required=False, default=default_file
        )
        parser.add_argument(
            "--samplerate",
            help="log sampling rate (Hz)",
            required=False,
            type=float,
            default=1,
        )
        parser.add_argument(
            "--default-speed",
            help="default leg speed for vehicle",
            required=False,
            default=None,
            action="store",
            dest="default_speed",
            type=float,
        )
        parser.add_argument(
            "--look-at-heading",
            help="heading to maintain while flying, if set. attitude is autopilot controlled if not set",
            required=False,
            default=None,
            action="store",
            dest="default_heading",
            type=float,
        )
        args = parser.parse_args(args=extra_args)

        self._pinging = not args.ping
        self._sampling = args.skipoutput
        self._sampling_delay = 1 / args.samplerate
        self._waypoint_fname = args.file

        if args.default_speed != None:
            self._default_leg_speed = args.default_speed
        if args.default_heading != None:
            self._default_heading = args.default_heading

        if self._sampling:
            self._log_file = open(args.output, "w+")
            self._cur_line = sum(1 for _ in self._log_file) + 1
            self._csv_writer = csv.writer(self._log_file)

    _ping_regex = re.compile(r".+icmp_seq=(?P<seq>\d+).+time=(?P<time>\d\.\d+) ms")

    async def _ping_latency(self, address: str, count: int):
        """
        This function will calculate the average latency to `address` by using
        ping and looking at the time in the output. This makes use of native
        aerpawlib tooling (ExternalProcess)
        """
        ping = ExternalProcess("ping", params=[address, "-c", str(count)])
        await ping.start()
        latencies = []
        # repeatedly wait for ping to produce output w/ icmp_seq field
        buff = 1
        while buff:
            buff = await ping.wait_until_output(r"icmp_seq=")
            ping_re_match = self._ping_regex.match(
                buff[-1]
            )  # last line contains useful data
            latencies.append(float(ping_re_match.group("time")))
            if ping_re_match.group("seq") == str(
                count
            ):  # if icmp_seq shows we've sent everything
                break
        avg_latency = sum(latencies) / len(latencies)
        return avg_latency

    @at_init
    async def ping_before_running(self, _):
        # do a few pings before waiting for the drone to arm
        if self._pinging:
            avg_ping_latency = await self._ping_latency(
                "127.0.0.1", 5
            )  # ping 127.0.0.1 5 times
            print(f"Average ping latency: {avg_ping_latency}ms")

    def _dump_to_csv(self, vehicle: Vehicle, line_num: int, writer):
        """
        This function will continually log stats about the vehicle to a file specified by command line args
        """
        pos = vehicle.position
        lat, lon, alt = pos.lat, pos.lon, pos.alt
        volt = vehicle.battery.voltage
        blevel = vehicle.battery.level
        timestamp = datetime.datetime.now()
        gps = vehicle.gps
        fix, num_sat = gps.fix_type, gps.satellites_visible
        if fix < 2:
            lat, lon, alt = -999, -999, -999
        vel = vehicle.velocity
        attitude = vehicle.attitude
        attitude_str = (
            "("
            + ",".join(map(str, [attitude.pitch, attitude.yaw, attitude.roll]))
            + ")"
        )
        writer.writerow(
            [line_num, lon, lat, alt, attitude_str, vel, volt, timestamp, fix, num_sat]
        )

    @background
    async def periodic_dump(self, vehicle: Vehicle):
        await sleep(self._sampling_delay)
        if not self._sampling:
            return
        self._dump_to_csv(vehicle, self._cur_line, self._csv_writer)
        self._log_file.flush()
        os.fsync(self._log_file)
        self._cur_line += 1

    def cleanup(self):
        if self._sampling:
            self._log_file.close()

    @state(name="take_off", first=True)
    async def take_off(self, vehicle: Vehicle):
        # take off to the alt of the first waypoint

        default_speed = 5 if isinstance(vehicle, Drone) else 1
        if self._default_leg_speed != None:
            default_speed = self._default_leg_speed

        self._waypoints = read_from_plan_complete(self._waypoint_fname, default_speed)

        if isinstance(vehicle, Drone):
            takeoff_alt = self._waypoints[self._current_waypoint]["pos"][2]
            print(f"Taking off to {takeoff_alt}m")
            await vehicle.takeoff(takeoff_alt)
        return "next_waypoint"

    @state(name="next_waypoint")
    async def next_waypoint(self, vehicle: Vehicle):
        # figure out the next waypoint to go to
        self._current_waypoint += 1
        if self._current_waypoint >= len(self._waypoints):
            return "rtl"
        print(f"Waypoint {self._current_waypoint}")
        waypoint = self._waypoints[self._current_waypoint]
        if waypoint["command"] == 20:  # RTL encountered, finish routine
            return "rtl"

        # go to next waypoint
        coords = Coordinate(*waypoint["pos"])
        target_speed = waypoint["speed"]
        in_background(
            vehicle.goto_coordinates(coords, target_heading=self._default_heading)
        )
        await asyncio.sleep(
            0.5
        )  # TODO to deal with MAV_CMD_DO_CHANGE_SPEED race condition -- needs field testing!
        await vehicle.set_groundspeed(target_speed)
        return "in_transit"

    @state(name="in_transit")
    async def in_transit(self, vehicle: Vehicle):
        # wait for the vehicle to arrive at the next waypoint and then transition

        # also as an example, measure ping latency while on the move
        if self._pinging:
            avg_ping_latency = await self._ping_latency(
                "127.0.0.1", 5
            )  # ping 127.0.0.1 5 times
            print(f"Average ping latency: {avg_ping_latency}ms")

        await vehicle.await_ready_to_move()
        return "at_waypoint"

    @timed_state(name="at_waypoint", duration=3)
    async def at_waypoint(self, _):
        # perform any extra functionality to be done at a waypoint, but stay
        # there for at least 3 seconds

        # ensure that we wait for some amount of time if specified in the .plan
        # default for no waiting in QGC is 0s, so this await wouldn't run
        wait_for = self._waypoints[self._current_waypoint]["wait_for"]
        if wait_for > 0:
            await sleep(wait_for)

        # example: measure average ping latency
        if self._pinging:
            avg_ping_latency = await self._ping_latency(
                "127.0.0.1", 5
            )  # ping 127.0.0.1 5 times
            print(f"Average ping latency: {avg_ping_latency}ms")

        return "next_waypoint"

    @state(name="rtl")
    async def rtl(self, vehicle: Vehicle):
        # return to the take off location and stop the script
        home_coords = Coordinate(
            vehicle.home_coords.lat, vehicle.home_coords.lon, vehicle.position.alt
        )
        await vehicle.goto_coordinates(
            home_coords, target_heading=self._default_heading
        )
        if isinstance(vehicle, Drone):
            await vehicle.land()
        print("done!")
