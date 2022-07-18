"""
Example of a script that has a single entry point that doesn't use any kind of
special Runner.
"""

from aerpawlib.runner import BasicRunner, entrypoint
from aerpawlib.util import VectorNED
from aerpawlib.vehicle import Drone

class MyScript(BasicRunner):
    @entrypoint
    async def do_stuff(self, drone: Drone):
        # take off to 10m
        await drone.takeoff(10)

        # fly north 10m
        await drone.goto_coordinates(drone.position + VectorNED(10, 0))

        # land
        await drone.land()
