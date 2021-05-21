"""
demo runner to show a possible implementation of a state machine for profiles
this also lets us sandbox things to a degree by using a custom vehicle

usage:
    python runner.py --script <python module path to experimenter script>

example:
    run basic multi-state script using the state machine
        python runner.py --script experimenter.experimenter_script
    
    run script that refuses to use state transitions
        python runner.py --script experimenter.statemachineless
"""

from aerpaw.staterunner import StateMachine
from aerpaw.vehicle import Vehicle
import importlib
import inspect

if __name__ == "__main__":
    from argparse import ArgumentParser

    parser = ArgumentParser()
    parser.add_argument("--script", help="experimenter script import", required=True)
    args = parser.parse_args()

    # import script and use reflection to get StateMachine
    experimenter_script = importlib.import_module(args.script)

    state_machine = None
    for _, val in inspect.getmembers(experimenter_script):
        if not inspect.isclass(val):
            continue
        if not issubclass(val, StateMachine):
            continue
        if val == StateMachine:
            continue
        state_machine = val()
        break

    # here's where we add our hooks or whatever else we want
    vehicle = Vehicle()

    # now run their code through our runner
    state_machine.run(vehicle)
