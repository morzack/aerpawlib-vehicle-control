import time
from aerpawlib.runner import StateMachine, state
from aerpawlib.util import calc_location_delta
from aerpawlib.vehicle import Drone

FLIGHT_ALT = 3 # m
SQUARE_SIZE = 10 # m
LOCATION_TOLERANCE = 1 # m

class SquareOff(StateMachine):
    @state(name="take_off", first=True)
    def take_off(self, drone: Drone):
        print("taking off")
        drone.takeoff(FLIGHT_ALT)
        drone.await_ready_to_move()
        drone.heading = 0
        return "leg_north"

    def perform_leg(self, drone: Drone, dNorth: float, dEast: float):
        current_pos = drone.position
        target_pos = calc_location_delta(current_pos, dNorth, dEast)
        drone.goto_coordinates(target_pos, tolerance=LOCATION_TOLERANCE)
        drone.await_ready_to_move()

    @state(name="leg_north")
    def leg_north(self, drone: Drone):
        print("heading north")
        self.perform_leg(drone, SQUARE_SIZE, 0)
        time.sleep(5)
        return "leg_west"
    
    @state(name="leg_west")
    def leg_west(self, drone: Drone):
        print("heading west")
        self.perform_leg(drone, 0, -SQUARE_SIZE)
        time.sleep(5)
        return "leg_south"
    
    @state(name="leg_south")
    def leg_south(self, drone: Drone):
        print("heading south")
        self.perform_leg(drone, -SQUARE_SIZE, 0)
        time.sleep(5)
        return "leg_east"
    
    @state(name="leg_east")
    def leg_east(self, drone: Drone):
        print("heading east")
        self.perform_leg(drone, 0, SQUARE_SIZE)
        time.sleep(5)
        return "land"

    @state(name="land")
    def land(self, drone: Drone):
        drone.land()
        # wait for disarm
        print("done!")
