import time
from aerpawlib.runner import StateMachine, state
from aerpawlib.util import calc_location_delta
from aerpawlib.vehicle import Drone, Rover, Vehicle

FLIGHT_ALT = 3 # m
SQUARE_SIZE = 10 # m
LOCATION_TOLERANCE = 2 # m -- ~2 is safe in general, use 3 for the rover in SITL

class SquareOff(StateMachine):
    @state(name="start", first=True)
    def start(self, vehicle: Vehicle):
        # wait for the vehicle to be armed 
        while not vehicle.armed: pass
        if isinstance(vehicle, Drone):
            return "take_off"
        elif isinstance(vehicle, Rover):
            return "leg_north"
        raise Exception("Vehicle not supported")

    @state(name="take_off")
    def take_off(self, drone: Drone):
        print("taking off")
        drone.takeoff(FLIGHT_ALT)
        drone.await_ready_to_move()
        drone.heading = 0
        drone.await_ready_to_move()
        return "leg_north"

    def perform_leg(self, vehicle: Vehicle, dNorth: float, dEast: float):
        current_pos = vehicle.position
        target_pos = calc_location_delta(current_pos, dNorth, dEast)
        vehicle.goto_coordinates(target_pos, tolerance=LOCATION_TOLERANCE)
        vehicle.await_ready_to_move()

    @state(name="leg_north")
    def leg_north(self, vehicle: Vehicle):
        print("heading north")
        self.perform_leg(vehicle, SQUARE_SIZE, 0)
        time.sleep(5)
        return "leg_west"
    
    @state(name="leg_west")
    def leg_west(self, vehicle: Vehicle):
        print("heading west")
        self.perform_leg(vehicle, 0, -SQUARE_SIZE)
        time.sleep(5)
        return "leg_south"
    
    @state(name="leg_south")
    def leg_south(self, vehicle: Vehicle):
        print("heading south")
        self.perform_leg(vehicle, -SQUARE_SIZE, 0)
        time.sleep(5)
        return "leg_east"
    
    @state(name="leg_east")
    def leg_east(self, vehicle: Vehicle):
        print("heading east")
        self.perform_leg(vehicle, 0, SQUARE_SIZE)
        time.sleep(5)
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
