"""
demo runner to show a possible implementation of a state machine for profiles
this also lets us sandbox things to a degree by using a custom vehicle

usage:
    python runner.py --script <script import path> --conn <connection string>

example:
    python -m aerpawlib --script experimenter_script --conn /dev/ttyACM0
"""

from .runner import BasicRunner, StateMachine, _Runner
from .vehicle import Vehicle

import importlib
import inspect

if __name__ == "__main__":
    from argparse import ArgumentParser

    parser = ArgumentParser(description="aerpawlib - wrap and run aerpaw scripts")
    parser.add_argument("--script", help="experimenter script", required=True)
    parser.add_argument("--conn", help="connection string", required=True)
    args = parser.parse_args()

    # import script and use reflection to get StateMachine
    experimenter_script = importlib.import_module(args.script)

    runner = None
    for _, val in inspect.getmembers(experimenter_script):
        if not inspect.isclass(val):
            continue
        if not issubclass(val, _Runner):
            continue
        if val in [StateMachine, BasicRunner]:
            continue
        if runner:
            raise Exception("You can only define one runner")
        runner = val()

    # here's where we add our hooks or whatever else we want
    vehicle = Vehicle()

    # now run their code through our runner
    runner.run(vehicle)
