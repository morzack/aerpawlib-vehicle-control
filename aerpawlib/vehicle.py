"""
Core logic surrounding the various `Vehicle`s available to aerpawlib user
scripts
"""
import asyncio
import dronekit
from pymavlink import mavutil
import time
from typing import Callable

from . import util

# time to wait when polling for dronekit vehicle state changes
_POLLING_DELAY = 0.01 # s

class Vehicle:
    """
    Overarching "generic vehicle" type. Implements all functionality, excluding
    movement commands (which are *always* vehicle specific).
    """
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

    _home_location: util.Coordinate

    def __init__(self, connection_string: str):
        self._vehicle = dronekit.connect(connection_string, wait_ready=True)
        
        # TODO this is commented until the filter is made more permissive
        # or we find an alternative way of getting the autopilot's home location
        # self._vehicle.commands.download()
        # self._vehicle.commands.wait_ready() # we need to do this to capture
        #                                     # things such as the home location
        
        self._has_heartbeat = False
        
        # register required listeners after connecting
        def _heartbeat_listener(_, __, value):
            if value > 1 and self._has_heartbeat:
                self._has_heartbeat = False
            elif value < 1 and not self._has_heartbeat:
                self._has_heartbeat = True
        self._vehicle.add_attribute_listener("last_heartbeat", _heartbeat_listener)

        def _abort_listener(_, __, value):
            # TODO abort logic is more complicated :P
            # if value != "GUIDED":
            #     self._abort()
            return
        self._vehicle.add_attribute_listener("mode", _abort_listener)

        # wait for connection
        while not self._has_heartbeat:
            time.sleep(_POLLING_DELAY)

    # nouns
    @property
    def connected(self) -> bool:
        """
        True if receiving heartbeats, False otherwise
        """
        return self._has_heartbeat

    @property
    def position(self) -> util.Coordinate:
        """
        Get the current position of the Vehicle as a `util.Coordinate`
        """
        loc = self._vehicle.location.global_relative_frame
        return util.Coordinate(loc.lat, loc.lon, loc.alt)

    @property
    def battery(self) -> dronekit.Battery:
        """
        Get the status of the battery. Wraps `dronekit.Battery`, which makes
        the `voltage`, `current`, and `level` available
        """
        return self._vehicle.battery

    @property
    def gps(self) -> dronekit.GPSInfo:
        """
        Get the current GPS status (for gps_0 -- can be changed in the future).
        Wraps `dronekit.GPSInfo`, which exposes the `fix_type` (0-1: no fix,
        2: 2d fix, 3: 3d fix), and number of `satellites_visible`, among other
        things.
        """
        return self._vehicle.gps_0

    @property
    def armed(self) -> bool:
        return self._vehicle.armed

    @property
    def home_coords(self) -> util.Coordinate:
        return self._home_location
    
    @property
    def heading(self) -> float:
        return self._vehicle.heading
    
    # special things
    def done_moving(self) -> bool:
        """
        See if the vehicle is ready to move (i.e. if the last movement command
        has been completed). Also makes sure that the vehicle is connected and
        that we haven't aborted.
        
        This is more accurately a function that describes the vehicle's
        willingness to take a new command.
        """
        if not self.connected or self._aborted:
            return False

        # syntax hack. functions and methods are different and need to be called
        # differently to prevent them from being bound to self
        if hasattr(self._ready_to_move, "__func__"):        # method
            return self._ready_to_move.__func__(self)
        return self._ready_to_move(self)                    # function
    
    async def await_ready_to_move(self):
        """
        Helper function that blocks execution and waits for the vehicle to
        finish the current action/movement that it was instructed to do.

        Makes use of `Vehicle.done_moving`
        """
        while not self.done_moving(): await asyncio.sleep(_POLLING_DELAY)

    def _abort(self):
        # TODO this should be something different in the future.
        # the intent of it in the past has been blocking further execution of
        # more vehicle control logic.
        if self._abortable:
            print("Aborted.")
            self._abortable = False
            self._aborted = True

    # verbs
    def close(self):
        """
        Clean up the `Vehicle` object/any state
        """
        self._vehicle.close()

    async def set_armed(self, value: bool):
        """
        Arm or disarm this vehicle, and wait for it to be armed (if possible)

        Dronekit doesn't guarentee that the vehicle arms immediately (or at
        all!), so this will block execution until the vehicle has been armed.

        If the vehicle can't be armed, an Exception is raised.
        """
        # dronekit doesn't guarentee that the vehicle arms immediately (or at all!)
        # this pattern keeps the funky logic out of the experimenter's script
        # to make sure that things are safer overall
        if not self._vehicle.is_armable:
            raise Exception("Not ready to arm") # in this case, the script dies completely
                                                # obviously not optimal *unless* we are
                                                # certain that a scipt always arms once
        self._vehicle.armed = value
        while not self._vehicle.armed: await asyncio.sleep(_POLLING_DELAY)

    def _initialize(self):
        """
        Generic pre-mission manipulation of the vehicle into a state that is
        acceptable. MUST be called before anything else. Though this is done by
        the runner.
        """
        while not self.armed: time.sleep(_POLLING_DELAY)

        self._vehicle.mode = dronekit.VehicleMode("GUIDED")
        self._abortable = True
        self._home_location = self.position
    
    async def goto_coordinates(self, coordinates: util.Coordinate, tolerance: float=2):
        """
        Make the vehicle go to provided coordinates.

        `tolerance` is the min distance away from the coordinates, in meters,
        that is acceptable.

        This method is only available for vehicles built off the `Vehicle` type
        (ex: `Drone` or `Rover`)
        """
        raise Exception("Generic vehicles can't go to coordinates!")

class Drone(Vehicle):
    """
    Drone vehicle type. Implements all functionality that AERPAW's drones
    expose to user scripts, which includes basic movement control (going to
    coords, turning, landing).
    """
    async def set_heading(self, heading: float):
        """
        Set the heading of the vehicle (in absolute deg).
        
        To turn a relative # of degrees, you can do something like
        `set_heading(drone.pos + x)`

        NOTE that this function still needs to be tested kind of extensively.
        Ardupilot has a few internal states that control the heading, and when
        it's manually set via a CMD_CONDITION_YAW command over mavlink, it's
        possible for it to either be ignored, be accepted and then ignored, or
        be accepted, switch the drone's internal turning state to be manually
        controlled, and then be stuck that way (i.e. the drone won't auto-fly
        in a "straight" direction). Basically, be warned.
        """
        await self.await_ready_to_move()

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

        while not _pointed_at_heading(self): await asyncio.sleep(_POLLING_DELAY)

    async def takeoff(self, target_alt: float, min_alt_tolerance: float=0.95):
        """
        Make the drone take off to a specific altitude, and blocks until the
        drone has reached that altitude.

        Additionally waits to make sure that channel 4 of RCIN (used for yaw)
        is centered to avoid yaw during takeoff if the drone was *just* armed.
        """
        await self.await_ready_to_move()        

        # TODO the below logic needs to be tested at the field (and likely made less brittle)
        # wait for sticks to return to center by taking rolling avg (30 frames)
        rcin_4 = [-999] * 30 # use something obviously out of range
        def _rcin_4_listener(_, __, message):
            rcin_4.pop(0)
            rcin_4.append(message.chan4_raw)
        self._vehicle.add_message_listener("RC_CHANNELS", _rcin_4_listener)
        while not 1450 <= (sum(rcin_4) / len(rcin_4)) <= 1550: await asyncio.sleep(_POLLING_DELAY)
        self._vehicle.remove_message_listener("RC_CHANNELS", _rcin_4_listener)
        
        self._vehicle.simple_takeoff(target_alt)
        
        taken_off = lambda self: self.position.alt >= target_alt * min_alt_tolerance
        self._ready_to_move = taken_off

        while not taken_off(self): await asyncio.sleep(_POLLING_DELAY)

    async def land(self):
        """
        Land the drone at its current position and block while waiting for it
        to be disarmed. No further movement is allowed after the drone has been
        landed (for now, may be changed later).
        """
        await self.await_ready_to_move()

        self._abortable = False
        self._vehicle.mode = dronekit.VehicleMode("LAND")

        self._ready_to_move = lambda _: False
        while self.armed: await asyncio.sleep(_POLLING_DELAY)

    async def goto_coordinates(self, coordinates: util.Coordinate, tolerance: float=2):
        await self.await_ready_to_move()
        self._vehicle.simple_goto(coordinates.location())
        
        # TODO in the future we likely want to split alt into a different tolerance
        at_coords = lambda self: \
            coordinates.distance(self.position) <= tolerance
        self._ready_to_move = at_coords

        while not at_coords(self): await asyncio.sleep(_POLLING_DELAY)

class Rover(Vehicle):
    """
    Rover vehicle type. Implements all functionality that AERPAW's rovers
    expose to user scripts, which includes basic movement control (going to
    coords).
    """
    async def goto_coordinates(self, coordinates: util.Coordinate, tolerance: float=2):
        await self.await_ready_to_move()
        self._vehicle.simple_goto(util.Coordinate(coordinates.lat, coordinates.lon, 0))
        
        at_coords = lambda self: \
            coordinates.ground_distance(self.position) <= tolerance
        self._ready_to_move = at_coords

        while not at_coords(self): await asyncio.sleep(_POLLING_DELAY)

# TODO break this down further:
# class LAM(Drone)
# class SAM(Drone)
