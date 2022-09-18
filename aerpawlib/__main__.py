"""
Tool used to run aerpawlib scripts that make use of a Runner class.

usage:
    python -m aerpawlib --script <script import path> --conn <connection string> \
            --vehicle <vehicle type>

example:
    python -m aerpawlib --script experimenter_script --conn /dev/ttyACM0 \
            --vehicle drone
"""

from .runner import BasicRunner, StateMachine, Runner, ZmqStateMachine
from .vehicle import Drone, Rover, Vehicle, DummyVehicle, VehicleConstraints
from .zmqutil import run_zmq_proxy

import asyncio
import importlib
import inspect
import yaml

def parse_config(config_data) -> VehicleConstraints:
    constraints = VehicleConstraints()

    if "velocity" in config_data:
        constraints.max_velocity = config_data["velocity"].get("max_velocity")
        constraints.min_velocity = config_data["velocity"].get("min_velocity")
    if "firmware" in config_data:
        constraints.ardupilot_version = config_data["firmware"].get("ardupilot_version")

    return constraints

async def _rtl_cleanup(vehicle: Vehicle):
    await vehicle.goto_coordinates(vehicle._home_location)
    if vehicle_type in [Drone]:
        await vehicle.land()

if __name__ == "__main__":
    from argparse import ArgumentParser

    parser = ArgumentParser(description="aerpawlib - wrap and run aerpaw scripts")
    parser.add_argument("--script", help="experimenter script", required=True)
    parser.add_argument("--conn", help="connection string", required=True)
    parser.add_argument("--vehicle", help="vehicle type [generic, drone, rover, none]", required=True)
    parser.add_argument("--skip-init", help="skip initialization", required=False,
            const=False, default=True, action="store_const", dest="initialize")
    parser.add_argument("--run-proxy", help="run zmq proxy", required=False,
            const=True, default=False, action="store_const", dest="run_zmq_proxy")
    parser.add_argument("--zmq-identifier", help="zmq identifier", required=False, dest="zmq_identifier")
    parser.add_argument("--zmq-proxy-server", help="zmq proxy server addr", required=False, dest="zmq_server_addr")
    parser.add_argument("--vehicle-config", help="vehicle specific configuration file with constraints",
            required=False, default=None, dest="vehicle_config_file")
    args, unknown_args = parser.parse_known_args() # we'll pass other args to the script

    if args.run_zmq_proxy:
        # don't even bother running the script, just the proxy
        run_zmq_proxy()
        exit()

    # import script and use reflection to get StateMachine
    experimenter_script = importlib.import_module(args.script)

    runner = None
    flag_zmq_runner = False
    for _, val in inspect.getmembers(experimenter_script):
        if not inspect.isclass(val):
            continue
        if not issubclass(val, Runner):
            continue
        if val in [StateMachine, BasicRunner, ZmqStateMachine]:
            continue
        if issubclass(val, ZmqStateMachine):
            flag_zmq_runner = True
        if runner:
            raise Exception("You can only define one runner")
        runner = val()

    vehicle_type = {
            "generic": Vehicle,
            "drone": Drone,
            "rover": Rover,
            "none": DummyVehicle
            }.get(args.vehicle, None)
    if vehicle_type is None:
        raise Exception("Please specify a valid vehicle type")
    vehicle = vehicle_type(args.conn)

    # handle loading the config and parsing any needed constraints to be applied
    if args.vehicle_config_file != None:
        with open(args.vehicle_config_file, 'r') as f:
            config_constraints = yaml.safe_load(f)
            vehicle._constraints = parse_config(config_constraints)

    # VV add any hooks that we want to the runner below VV
    
    # too bad there aren't any (yet) to use as an example

    # ^^                                                ^^

    runner.initialize_args(unknown_args)
    
    if vehicle_type in [Drone, Rover] and args.initialize:
        vehicle._initialize_prearm(args.initialize)

    if flag_zmq_runner:
        if None in [args.zmq_identifier, args.zmq_server_addr]:
            raise Exception("you must declare an identifier and server address for a zmq enabled state machine")
        print("initializing zmq bindings")
        runner._initialize_zmq_bindings(args.zmq_identifier, args.zmq_server_addr)

    asyncio.run(runner.run(vehicle))
    
    # rtl / land if not already done
    if vehicle_type in [Drone, Rover] and vehicle.armed:
        print("[aerpawlib] Vehicle still armed after experiment! RTLing and LANDing automatically.")
        asyncio.run(_rtl_cleanup(vehicle))

    # clean up
    vehicle.close()
