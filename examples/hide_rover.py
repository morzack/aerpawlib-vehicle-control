"""
hide_rover will make the vehicle used (a rover) take in a .plan file and .kml file (geofence), parse them, and make the vehicle
follow the path provided in the plan. After finishing the path, the rover will move to a random coordinate within the provided geofence and "hide".
The .plan file should move the rover to a location from which it will be able to move to anywhere within the geofence safely.
You can optionally provide a specific latitude & longitude to hide the rover at via arguments (see Usage).

Usage:
    python -m aerpawlib --conn ... --vehicle rover --script hide_rover \
            --file <.plan file to run> \
            --hide-fence <.kml file containing geofence> \
            --lat <latitude to hide rover (optional)> \
            --lon <longitude to hide rover (optional)>

State vis:

┌────────────┐
│ initialize │
└──────┬─────┘
       │
       ├────────────────────────────────────────┐
       │                                        │
┌──────▼────────┐    ┌────────────┐     ┌───────┴─────┐
│ next_waypoint ├────► in_transit ├─────► at_waypoint │
└──────┬────────┘    └────────────┘     └─────────────┘
       │
 ┌─────▼──────┐
 │ hide_rover │
 └────────────┘
"""

import asyncio
from argparse import ArgumentParser
import datetime
import re
import csv
import os
from typing import List, TextIO
import random

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
from aerpawlib.util import Coordinate, Waypoint, read_from_plan_complete, inside, readGeofence
from aerpawlib.vehicle import Drone, Rover, Vehicle


class HideRover(StateMachine):
    _waypoints = []
    _waypoint_fname: str
    _current_waypoint: int = 0

    _default_leg_speed: float = None
    _default_heading: float = None
    _hide_latitude: float = None
    _hide_longitude: float = None

    _next_sample: float = 0
    _sampling_delay: float
    _cur_line: int
    _csv_writer: object
    _log_file: TextIO
    _hide_geofence: list

    def initialize_args(self, extra_args: List[str]):
        # use an extra argument parser to read in custom script arguments
        default_file = (
            f"GPS_DATA_{datetime.datetime.now().strftime('%Y-%m-%d_%H:%M:%S')}.csv"
        )

        parser = ArgumentParser()
        parser.add_argument("--file", help="Mission plan file path.", required=True)
        parser.add_argument(
            "--skipoutput", help="don't dump gps data to a file", action="store_false"
        )
        parser.add_argument(
            "--output", help="log output file", required=False, default=default_file
        )
        parser.add_argument(
            "--samplerate", help="log sampling rate (Hz)", required=False, default=1
        )
        parser.add_argument(
            "--default-speed",
            help="default leg speed for vehicle",
            required=False,
            default=None,
            action="store",
            dest="default_speed",
        )
        parser.add_argument(
            "--lat",
            type=float,
            help="latitude to hide rover at",
            required=False,
            default=None,
            action="store",
            dest="latitude",
        )
        parser.add_argument(
            "--lon",
            type=float,
            help="longitude to hide rover at",
            required=False,
            default=None,
            action="store",
            dest="longitude",
        )
        parser.add_argument(
            "--hide-fence",
            type=str,
            help="geofence to hide the rover in",
            required=True,
            default=None,
            action="store",
            dest="geofence_file_name",
        )
        args = parser.parse_args(args=extra_args)

        self._sampling = args.skipoutput
        self._sampling_delay = 1 / args.samplerate
        self._waypoint_fname = args.file
        self._hide_geofence = readGeofence(args.geofence_file_name)

        if args.default_speed != None:
            self._default_leg_speed = args.default_speed
        if args.latitude != None:
            self._hide_latitude = args.latitude
        if args.longitude != None:
            self._hide_longitude = args.longitude

        if self._sampling:
            self._log_file = open(args.output, "w+")
            self._cur_line = sum(1 for _ in self._log_file) + 1
            self._csv_writer = csv.writer(self._log_file)

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

    def generate_random_coordinate(self, geofence: list):
        # generates a random coordinate within the provided geofence

        # creates a maximum square that completely covers the geofence,
        # then generates random coordinates within this square until a coordinate that is within the geofence is generated
        min_lat, max_lat, min_lon, max_lon = geofence[0]['lat'], geofence[0]['lat'], geofence[0]['lon'], geofence[0]['lon']
        for coord in geofence:
            min_lat = min(min_lat, coord['lat'])
            max_lat = max(max_lat, coord['lat'])
            min_lon = min(min_lon, coord['lon'])
            max_lon = max(max_lon, coord['lon'])

        random_lat, random_lon = random.uniform(min_lat, max_lat), random.uniform(min_lon, max_lon)
        while not inside(random_lon, random_lat, geofence):
            random_lat, random_lon = random.uniform(min_lat, max_lat), random.uniform(min_lon, max_lon)

        return Coordinate(random_lat, random_lon)

    @state(name="init_state", first=True)
    async def initialize(self, vehicle: Vehicle):
        # initialize parameters needed for running and read waypoints from plan
        default_speed = 1
        if self._default_leg_speed != None:
            default_speed = self._default_leg_speed

        self._waypoints = read_from_plan_complete(self._waypoint_fname, default_speed)

        # check if hide_latitude and hide_longitude are inside geofence if specified via arguments
        if self._hide_latitude != None and self._hide_longitude != None and not inside(self._hide_longitude, self._hide_latitude, self._hide_geofence):
            print("WARNING: Specified latitude, longitude not inside provided geofence.")

        # checks if only one coordinate was specified via arguments (this is invalid, so the script will stop)
        if (self._hide_latitude == None) ^ (self._hide_longitude == None):
            print("Only one coordinate unit was specified (either latitude or longitude). Please specify either both or neither.\nStopping script")
            return None

        # generate random hide coords if not specified
        if self._hide_latitude == None and self._hide_longitude == None:
            random_coord = self.generate_random_coordinate(self._hide_geofence)
            self._hide_latitude = random_coord.lat
            self._hide_longitude = random_coord.lon

        print("Hiding at Latitude: ", self._hide_latitude)
        print("Hiding at Longitude: ", self._hide_longitude)

        return "next_waypoint"

    @state(name="next_waypoint")
    async def next_waypoint(self, vehicle: Vehicle):
        # figure out the next waypoint to go to
        self._current_waypoint += 1
        # if last waypoint has been reached, transition to hide_rover state
        if self._current_waypoint >= len(self._waypoints):
            return "hide_rover"
        print(f"Waypoint {self._current_waypoint}")
        waypoint = self._waypoints[self._current_waypoint]

        # go to next waypoint
        coords = Coordinate(*waypoint["pos"])
        target_speed = waypoint["speed"]
        await vehicle.set_groundspeed(target_speed)
        in_background(
            vehicle.goto_coordinates(coords, target_heading=self._default_heading)
        )
        return "in_transit"

    @state(name="in_transit")
    async def in_transit(self, vehicle: Vehicle):
        # wait for the vehicle to arrive at the next waypoint and then transition
        await vehicle.await_ready_to_move()
        return "at_waypoint"

    @timed_state(name="at_waypoint", duration=3)
    async def at_waypoint(self, _):
        # ensure that we wait for some amount of time if specified in the .plan
        # default for no waiting in QGC is 0s, so this await wouldn't run
        wait_for = self._waypoints[self._current_waypoint]["wait_for"]
        if wait_for > 0:
            await sleep(wait_for)

        return "next_waypoint"

    @state(name="hide_rover")
    async def hide_rover(self, vehicle: Vehicle):
        # head to hiding location and stop the script
        print("Hiding")
        hide_coords = Coordinate(self._hide_latitude, self._hide_longitude, vehicle.position.alt)
        await vehicle.goto_coordinates(
            hide_coords, target_heading=self._default_heading
        )
        print("done!")
        return None
