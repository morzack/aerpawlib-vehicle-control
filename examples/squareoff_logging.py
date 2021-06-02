from argparse import ArgumentParser
import csv
import datetime
import time
from typing import List, TextIO
from aerpawlib.runner import StateMachine, state, background
from aerpawlib.util import calc_location_delta
from aerpawlib.vehicle import Drone, Rover, Vehicle

FLIGHT_ALT = 3 # m
SQUARE_SIZE = 10 # m
LOCATION_TOLERANCE = 3 # m -- ~2 is safe in general, use 3 for the rover in SITL
WAIT_TIME = 5 # s

def _dump_to_csv(vehicle: Vehicle, line_num: int, writer):
    pos = vehicle.position
    lat, lon, alt = pos.lat, pos.lon, pos.alt
    volt = vehicle.battery.voltage
    timestamp = datetime.datetime.now()
    gps = vehicle.gps
    fix, num_sat = gps.fix_type, gps.satellites_visible
    if fix < 2:
        lat, lon, alt = -999, -999, -999
    writer.writerow([line_num, lon, lat, alt, volt, timestamp, fix, num_sat])

class SquareOff(StateMachine):
    def initialize_args(self, extra_args: List[str]):
        default_file = f"GPS_DATA_{datetime.datetime.now().strftime('%Y-%m-%d_%H:%M:%S')}.csv"

        parser = ArgumentParser()
        parser.add_argument("--output", help="log output file", required=False,
                default=default_file)
        parser.add_argument("--samplerate", help="log sampling rate (Hz)", required=False,
                default=1)
        args = parser.parse_args(args=extra_args)
    
        self._sampling_delay = 1 / args.samplerate
        self._log_file = open(args.output, 'w+')
        self._cur_line = sum(1 for _ in self._log_file) + 1
        self._csv_writer = csv.writer(self._log_file)

    _next_sample: float=0
    _sampling_delay: float
    _cur_line: int
    _csv_writer: object
    _log_file: TextIO
    
    @background
    def periodic_dump(self, vehicle: Vehicle):
        if time.time() > self._next_sample:
            self._next_sample = time.time() + self._sampling_delay
            if not vehicle.connected:
                return
            _dump_to_csv(vehicle, self._cur_line, self._csv_writer)
            self._cur_line += 1

    def cleanup(self):
        self._log_file.close()
    
    _legs = ["leg_north", "leg_west", "leg_south", "leg_east"]
    _current_leg = 0

    @state(name="start", first=True)
    def start(self, vehicle: Vehicle):
        if isinstance(vehicle, Drone):
            return "take_off"
        elif isinstance(vehicle, Rover):
            return self._legs[self._current_leg]
        raise Exception("Vehicle not supported")

    @state(name="take_off")
    def take_off(self, drone: Drone):
        print("taking off")
        drone.takeoff(FLIGHT_ALT)
        drone.await_ready_to_move()
        drone.heading = 0
        drone.await_ready_to_move()
        return "leg_north"

    def command_leg(self, vehicle: Vehicle, dNorth: float, dEast: float):
        current_pos = vehicle.position
        target_pos = calc_location_delta(current_pos, dNorth, dEast)
        vehicle.goto_coordinates(target_pos, tolerance=LOCATION_TOLERANCE)
        vehicle.await_ready_to_move()

    @state(name="in_transit")
    def in_transit(self, vehicle: Vehicle):
        if not vehicle.done_moving():
            return "in_transit"
        self._position_timer = time.time() + WAIT_TIME
        return "at_position"

    _position_timer: float=0
    
    @state(name="at_position")
    def at_position(self, _):
        # make sure that we wait at each corner
        if time.time() <= self._position_timer:
            return "at_position"
        self._current_leg += 1
        if self._current_leg < len(self._legs):
            return self._legs[self._current_leg]
        return "finish"

    @state(name="leg_north")
    def leg_north(self, vehicle: Vehicle):
        print("heading north")
        self.command_leg(vehicle, SQUARE_SIZE, 0)
        return "in_transit"
    
    @state(name="leg_west")
    def leg_west(self, vehicle: Vehicle):
        print("heading west")
        self.command_leg(vehicle, 0, -SQUARE_SIZE)
        return "in_transit"
    
    @state(name="leg_south")
    def leg_south(self, vehicle: Vehicle):
        print("heading south")
        self.command_leg(vehicle, -SQUARE_SIZE, 0)
        return "in_transit"
    
    @state(name="leg_east")
    def leg_east(self, vehicle: Vehicle):
        print("heading east")
        self.command_leg(vehicle, 0, SQUARE_SIZE)
        return "in_transit"

    @state(name="finish")
    def finish(self, vehicle: Vehicle):
        if isinstance(vehicle, Drone):
            return "land"
        elif isinstance(vehicle, Rover):
            print("done!")
            # remember that returning nothing == script over

    @state(name="land")
    def land(self, drone: Drone):
        drone.land()
        # wait for disarm
        print("done!")
