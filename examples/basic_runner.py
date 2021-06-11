"""
Example of a script that has a single entry point that doesn't use any kind of
special Runner.
"""

from aerpawlib.runner import BasicRunner, entrypoint
from aerpawlib.util import VectorNED
from aerpawlib.vehicle import Drone

class MyScript(BasicRunner):
    @entrypoint
    def do_stuff(self, drone: Drone):
        # take off to 10m
        drone.takeoff(10)

        # wait until done taking off (to make sure coords in next step are right)
        drone.await_ready_to_move()

        # fly north 10m
        drone.goto_coordinates(drone.position + VectorNED(10, 0))

        # land
        drone.land()
