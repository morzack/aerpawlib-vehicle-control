"""
Example of a script that has a single entry point that doesn't use any kind of
special Runner.
"""

from aerpawlib.runner import BasicRunner, entrypoint
from aerpawlib.util import Coordinate, VectorNED
from aerpawlib.vehicle import Drone
import asyncio

class FlyRoutine(BasicRunner):
    @entrypoint
    async def start_flight(self, vehicle: Drone):
        print("Start Flight")
        await vehicle.takeoff(25)
        print("Goto TW2...")
        await vehicle.goto_coordinates(vehicle.position + VectorNED(-16, 8, 0))
        await asyncio.sleep(10.0)
    
        print("Landing")
        await vehicle.land()

