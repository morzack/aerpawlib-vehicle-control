import dronekit
from pymavlink import mavutil
from time import sleep
from typing import Callable

from . import util

# time to wait when polling for dronekit vehicle state changes
_POLLING_DELAY = 0.01 # s

class Vehicle:
    _vehicle: dronekit.Vehicle
    _has_heartbeat: bool
    
    # function used by "verb" functions to check and see if the vehicle can be
    # commanded to move. should be set to a new closure by verb functions to
    # redefine functionality
    _ready_to_move: Callable[[object], bool]=lambda _: True

    # temp hack to allow for dynamically making the drone abortable or not
    # aborting is triggered by mode changes, so we need to ignore the initial
    # takeoff and final landing changes
    _abortable: bool=False
    _aborted: bool=False

    def __init__(self, connection_string: str):
        self._vehicle = dronekit.connect(connection_string, wait_ready=True)
        self._vehicle.commands.download()
        self._vehicle.commands.wait_ready() # we need to do this to capture
                                            # things such as the home location
        
        self._has_heartbeat = False
        
        # can be pulled out to go elsewhere later
        def _heartbeat_listener(_, __, value):
            if value > 1 and self._has_heartbeat:
                self._has_heartbeat = False
            elif value < 1 and not self._has_heartbeat:
                self._has_heartbeat = True
        self._vehicle.add_attribute_listener("last_heartbeat", _heartbeat_listener)

        def _abort_listener(_, __, value):
            if value != "GUIDED":
                self._abort()
        self._vehicle.add_attribute_listener("mode", _abort_listener)

        while not self._has_heartbeat:
            sleep(_POLLING_DELAY)

    # nouns
    @property
    def connected(self) -> bool:
        return self._has_heartbeat

    @property
    def position(self) -> dronekit.LocationGlobalRelative:
        return self._vehicle.location.global_relative_frame

    @property
    def battery(self) -> dronekit.Battery:
        return self._vehicle.battery

    @property
    def gps(self) -> dronekit.GPSInfo:
        return self._vehicle.gps_0

    @property
    def armed(self) -> bool:
        return self._vehicle.armed

    @armed.setter
    def armed(self, value):
        # dronekit doesn't guarentee that the vehicle arms immediately (or at all!)
        # this pattern keeps the funky logic out of the experimenter's script
        # to make sure that things are safer overall
        if not self._vehicle.is_armable:
            raise Exception("Not ready to arm")
        self._vehicle.armed = value
        while not self._vehicle.armed: sleep(_POLLING_DELAY)

    @property
    def home_coords(self) -> dronekit.LocationGlobalRelative:
        loc = self._vehicle.home_location
        return dronekit.LocationGlobalRelative(loc.lat, loc.lon)
    
    @property
    def heading(self) -> float:
        return self._vehicle.heading
    
    # special things
    def done_moving(self) -> bool:
        """
        See if the vehicle is ready to move (i.e. if the last movement command is complete)

        Also makes sure that the vehicle is connected and that we haven't aborted.
        
        This is more accurately a function that describes the vehicle's willingness
        to take a new command
        """
        if not self.connected or self._aborted:
            return False

        # mild syntax hack. basically there's a difference between "methods"
        # and "functions". we can directly call functions w/out worrying about
        # self, but we *have* to unbind methods.
        # this is complicated by the fact that we use both `def` and `lambda`
        # to set _ready_to_move in different situations (for syntactical clarity)
        if hasattr(self._ready_to_move, "__func__"):        # method
            return self._ready_to_move.__func__(self)
        return self._ready_to_move(self)                    # function
    
    def await_ready_to_move(self):
        """
        Helper blocking function that waits for the vehicle to finish the current
        action/movement that it was instructed to do
        """
        while not self.done_moving(): sleep(_POLLING_DELAY)

    def _abort(self):
        # TODO this should be something different in the future.
        # the intent of it in the past has been blocking further execution of
        # more vehicle control logic.
        if self._abortable:
            print("Aborted.")
            self._abortable = False
            self._aborted = True # TODO i'm not a fan of the flag approach here

    # verbs
    def close(self):
        self._vehicle.close()

    def _initialize(self):
        """
        Generic pre-mission manipulation of the vehicle into a state that is
        acceptable. MUST be called before anything else. Though this is done by
        the runner.
        """
        while not self.armed: sleep(_POLLING_DELAY)

        self._vehicle.mode = dronekit.VehicleMode("GUIDED")
        self._abortable = True

    def goto_coordinates(self, coordinates: dronekit.LocationGlobalRelative, tolerance: float=2):
        """
        Make the vehicle go to provided coordinates. Blocks while waiting for the
        vehicle to be ready to move

        tolerance is the min distance away from the coordinates, in meters, that are
        acceptable
        """
        self.await_ready_to_move()
        self._vehicle.simple_goto(coordinates)
        # NOTE fwiw: we've never done this in the past, but this should include alt probably
        # also, maybe use a rolling average in the future?
        self._ready_to_move = lambda self: \
            util.calc_distance(coordinates, self.position) <= tolerance

class Drone(Vehicle):
    @Vehicle.heading.setter
    def heading(self, heading: float):
        """
        Blocking way to set the heading of the vehicle (in absolute deg).
        Can be paired up with += to use relative coordinates -- i.e. we will
        never turn relative to our current heading
        """
        self.await_ready_to_move()

        heading %= 360
        
        # NOTE that the system and component below are derived from commands
        # observed in SITL. could be wrong, and it's kind of magic undocumnted stuff.
        # doing more research.
        msg = self._vehicle.message_factory.command_long_encode(
            1, 250,                                     # target system, component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,      # command
            0,                                          # confirmation
            heading,                                    # yaw angle in deg
            0,                                          # yaw speed in deg/s
            0,                                          # direction to turn in (-1: ccw, 1: cw)
            0,                                          # never turn relative to our current heading
            0, 0, 0                                     # unused
            )
        self._vehicle.send_mavlink(msg)
        
        def _pointed_at_heading(self) -> bool:
            _TURN_TOLERANCE_DEG = 5
            turn_diff = min([abs(i) for i in [heading - self.heading, self.heading - (heading + 360)]])
            return turn_diff <= _TURN_TOLERANCE_DEG
        self._ready_to_move = _pointed_at_heading

    def takeoff(self, target_alt: float, min_alt_tolerance: float=0.95):
        """
        Blocking function (waits for the drone to be ready to move) that makes
        a drone wait to be armed and then takes off to a specific altituide
        """
        self.await_ready_to_move()        

        # wait for sticks to return to center by taking rolling avg (30 frames)
        rcin_4 = [-999] * 30 # use something obviously out of range
        def _rcin_4_listener(_, __, message):
            rcin_4.pop(0)
            rcin_4.append(message.chan4_raw)
        self._vehicle.add_message_listener("RC_CHANNELS", _rcin_4_listener)
        while not 1450 <= (sum(rcin_4) / len(rcin_4)) <= 1550: sleep(_POLLING_DELAY)
        self._vehicle.remove_message_listener("RC_CHANNELS", _rcin_4_listener)
        
        self._vehicle.simple_takeoff(target_alt)
        
        self._ready_to_move = lambda self: self.position.alt >= target_alt * min_alt_tolerance

    def land(self):
        """
        Land the drone and wait for it to be disarmed (BLOCKING).
        No further movement is allowed (for now!)
        """
        self.await_ready_to_move()

        self._abortable = False
        self._vehicle.mode = dronekit.VehicleMode("LAND")

        self._ready_to_move = lambda _: False
        while self.armed: sleep(_POLLING_DELAY)

class Rover(Vehicle):
    def goto_coordinates(self, coordinates: dronekit.LocationGlobalRelative, tolerance: float=2):
        coords = dronekit.LocationGlobalRelative(coordinates.lat, coordinates.lon, 0)
        return super().goto_coordinates(coords, tolerance=tolerance)

# TODO break this down further:
# class LAM(Drone)
# class SAM(Drone)
