"""
Tool used to run aerpawlib scripts that make use of a Runner class.

usage:
    python -m aerpawlib --script <script import path> --conn <connection string> \
            --vehicle <vehicle type>

example:
    python -m aerpawlib --script experimenter_script --conn /dev/ttyACM0 \
            --vehicle drone
"""

from .runner import BasicRunner, StateMachine, Runner
from .vehicle import Drone, Rover, Vehicle

import asyncio
import importlib
import inspect

if __name__ == "__main__":
    from argparse import ArgumentParser

    parser = ArgumentParser(description="aerpawlib - wrap and run aerpaw scripts")
    parser.add_argument("--script", help="experimenter script", required=True)
    parser.add_argument("--conn", help="connection string", required=True)
    parser.add_argument("--vehicle", help="vehicle type [generic, drone, rover]", required=True)
    parser.add_argument("--skip-init", help="skip initialization", required=False,
            const=False, default=True, action="store_const", dest="initialize")
    args, unknown_args = parser.parse_known_args() # we'll pass other args to the script

    # import script and use reflection to get StateMachine
    experimenter_script = importlib.import_module(args.script)

    runner = None
    for _, val in inspect.getmembers(experimenter_script):
        if not inspect.isclass(val):
            continue
        if not issubclass(val, Runner):
            continue
        if val in [StateMachine, BasicRunner]:
            continue
        if runner:
            raise Exception("You can only define one runner")
        runner = val()

    vehicle_type = {
            "generic": Vehicle,
            "drone": Drone,
            "rover": Rover
            }.get(args.vehicle, None)
    if vehicle_type is None:
        raise Exception("Please specify a valid vehicle type")
    vehicle = vehicle_type(args.conn)

    # VV add any hooks that we want to the runner below VV
    
    # too bad there aren't any (yet) to use as an example

    # ^^                                                ^^

    runner.initialize_args(unknown_args)
    
    if vehicle_type in [Drone, Rover] and args.initialize:
        vehicle._initialize_prearm(args.initialize)
    
    asyncio.run(runner.run(vehicle))
    
    # rtl / land if not already done
    if vehicle_type in [Drone, Rover] and vehicle.armed:
        print("[aerpawlib] Vehicle still armed after experiment! RTLing and LANDing automatically.")
        vehicle.goto_coordinates(vehicle._home_location)
        if vehicle_type in [Drone]:
            vehicle.land()
    
    # clean up
    vehicle.close()
