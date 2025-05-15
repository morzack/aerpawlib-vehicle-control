"""
Tool used to run aerpawlib scripts that make use of a Runner class.

usage:
    python -m aerpawlib --script <script import path> --conn <connection string> \
            --vehicle <vehicle type>

example:
    python -m aerpawlib --script experimenter_script --conn /dev/ttyACM0 \
            --vehicle drone
"""

from .aerpaw import AERPAW_Platform
from .runner import BasicRunner, StateMachine, Runner, ZmqStateMachine
from .vehicle import Drone, Rover, Vehicle, DummyVehicle
from .zmqutil import run_zmq_proxy

import asyncio
import importlib
import inspect
import time

async def _rtl_cleanup(vehicle: Vehicle):
    await vehicle.goto_coordinates(vehicle._home_location)
    if vehicle_type in [Drone]:
        await vehicle.land()

if __name__ == "__main__":
    from argparse import ArgumentParser
    import sys
    
    proxy_mode = "--run-proxy" in sys.argv

    parser = ArgumentParser(description="aerpawlib - wrap and run aerpaw scripts")
    parser.add_argument("--script", help="experimenter script", required=not proxy_mode)
    parser.add_argument("--conn", help="connection string", required=not proxy_mode)
    parser.add_argument("--vehicle", help="vehicle type [generic, drone, rover, none]", required=not proxy_mode)
    parser.add_argument("--skip-init", help="skip initialization", required=False,
            const=False, default=True, action="store_const", dest="initialize")
    parser.add_argument("--run-proxy", help="run zmq proxy", required=False,
            const=True, default=False, action="store_const", dest="run_zmq_proxy")
    parser.add_argument("--zmq-identifier", help="zmq identifier", required=False, dest="zmq_identifier")
    parser.add_argument("--zmq-proxy-server", help="zmq proxy server addr", required=False, dest="zmq_server_addr")
    parser.add_argument("--skip-rtl", help="don't rtl and land at the end of an experiment automatically",
            const=False, default=True, action="store_const", dest="rtl_at_end")
    parser.add_argument("--debug-dump", help="run aerpawlib's internal debug dump on vehicle object", required=False,
            const=True, default=False, action="store_const", dest="debug_dump")
    parser.add_argument("--no-aerpawlib-stdout", help="prevent aerpawlib from printing to stdout", required=False,
            const=True, default=False, action="store_const", dest="no_stdout")
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
    
    if args.debug_dump and hasattr(vehicle, "_verbose_logging"):
        vehicle._verbose_logging = True

    AERPAW_Platform._no_stdout = args.no_stdout

    # everything after this point is user script dependent. avoid adding extra logic below here

    runner.initialize_args(unknown_args)
    
    if vehicle_type in [Drone, Rover] and args.initialize:
        vehicle._initialize_prearm(args.initialize)

    if flag_zmq_runner:
        if None in [args.zmq_identifier, args.zmq_server_addr]:
            raise Exception("you must declare an identifier and server address for a zmq enabled state machine")
        print("[aerpawlib] initializing zmq bindings")
        runner._initialize_zmq_bindings(args.zmq_identifier, args.zmq_server_addr)

    asyncio.run(runner.run(vehicle))
    
    # rtl / land if not already done
    if vehicle_type in [Drone, Rover]:
        if vehicle.armed and args.rtl_at_end:
            AERPAW_Platform.log_to_oeo("[aerpawlib] Vehicle still armed after experiment! RTLing and LANDing automatically.")
            asyncio.run(_rtl_cleanup(vehicle))

        stop_time = time.time()
        seconds_to_complete = int(stop_time - vehicle._mission_start_time)
        time_to_complete = f"{(seconds_to_complete // 60):02d}:{(seconds_to_complete % 60):02d}"
        AERPAW_Platform.log_to_oeo(f"[aerpawlib] Mission took {time_to_complete} mm:ss")

    # clean up
    vehicle.close()
