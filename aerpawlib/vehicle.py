"""
Core logic surrounding the various `Vehicle`s available to aerpawlib user
scripts
"""
import asyncio
import math
from dataclasses import dataclass
import dronekit
from pymavlink import mavutil
import time
from typing import Callable

from . import util

# time to wait when polling for dronekit vehicle state changes
_POLLING_DELAY = 0.01 # s

_YAW_BITMASK = 0b100111111111

class DummyVehicle:
    """
    vehicle for things that don't need vehicles :)

    hacky lol
    """
    
    def __init__(self, connection_string: str):
        pass

    def close(self):
        pass

    def _initialize_prearm(self, should_postarm_init):
        pass

    async def _initialize_postarm(self):
        pass

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

    # _current_heading is used to blend heading and velocity control commands
    # if a script sets the vehicle's heading, it will override _current_heading and be used
    # for all future commands that can provide a heading
    _current_heading: float=None

    def __init__(self, connection_string: str):
        self._vehicle = dronekit.connect(connection_string, wait_ready=True)
        
        self._vehicle.commands.download()
        self._vehicle.commands.wait_ready() # we need to do this to capture
                                            # things such as the home location
        
        # # this hack is needed to wait for the autopilot to reply with all needed info for the runner
        # while self.autopilot_info.major == None: time.sleep(0.5)
        
        self._has_heartbeat = False

        self._should_postarm_init = True

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

    @property
    def velocity(self) -> util.VectorNED:
        return util.VectorNED(*self._vehicle.velocity)

    @property
    def autopilot_info(self) -> dronekit.Version:
        return self._vehicle.version

    @property
    def attitude(self) -> dronekit.Attitude:
        """
        attitude of the vehicle, all values in radians

        - pitch/roll are horizon-relative
        - yaw is world relative (north=0)
        """
        return self._vehicle.attitude
    
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

        Additionally, will block and request arming if the vehicle isn't yet
        armed.
        """
        if not self.armed:
            await self._initialize_postarm() # also contains logic to wait for arm
        
        while not self.done_moving(): await asyncio.sleep(_POLLING_DELAY)

    def _abort(self):
        # TODO this should be something different in the future.
        # the intent of it in the past has been blocking further execution of
        # more vehicle control logic.
        if self._abortable:
            print("[aerpawlib] Aborted.")
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
        while self._vehicle.armed != value: await asyncio.sleep(_POLLING_DELAY)
    
    def _initialize_prearm(self, should_postarm_init):
        while not self._vehicle.system_status in ["STANDBY", "ACTIVE"]: time.sleep(_POLLING_DELAY)
        self._should_postarm_init = should_postarm_init

    async def _initialize_postarm(self):
        """
        Generic pre-mission manipulation of the vehicle into a state that is
        acceptable. MUST be called before anything else. Though this is done by
        the runner.
        """
        if not self._should_postarm_init:
            return

        print("[aerpawlib] Guided command attempted. Waiting for safety pilot to arm")
        while not self._vehicle.is_armable: await asyncio.sleep(_POLLING_DELAY)
        while not self.armed: await asyncio.sleep(_POLLING_DELAY)

        self._vehicle.mode = dronekit.VehicleMode("GUIDED")
        self._abortable = True
        self._home_location = self.position
    
    async def goto_coordinates(self,
            coordinates: util.Coordinate,
            tolerance: float=2,
            target_heading: float=None):
        """
        Make the vehicle go to provided coordinates.

        `tolerance` is the min distance away from the coordinates, in meters,
        that is acceptable.

        `target_heading`, when set, will make the drone point in the specified
        direction (absolute). Otherwise the drone will remain pointing in the
        current direction. This is only available on drones.

        This method is only available for vehicles built off the `Vehicle` type
        (ex: `Drone` or `Rover`)
        """
        raise Exception("Generic vehicles can't go to coordinates!")

    async def set_velocity(self,
            velocity_vector: util.VectorNED,
            global_relative: bool=True,
            duration: float=None):
        """
        Set a drone's velocity that it will use for `duration` seconds.

        The velocity vector provided will be inerpreted as being global
        relative *in direction* unless the `global_relative` parameter is set
        to False.

        The vehicle will maintain this velocity for `duration` seconds, if
        `duration` is provided, otherwise it will automatically maintain the
        specified velocity until another command is sent.

        The vehicle's velocity vector, if it has a magnitude greater than the
        configured "max_velocity" (configurable in aerpawlib by using a config.yaml
        file with the --vehicle-config argument, defaults to 5 m/s), will be
        normalized and rescaled such that the new magnitude is equal to the
        maximum velocity.
        """
        raise Exception("set_velocity not implemented")

    async def set_groundspeed(self, velocity: float):
        """
        Set a vehicle's cruise velocity as used by the autopilot when performing
        guided movement operations (ex: goto_coordinates). In m/s.

        NOTE:
            This is not always respected by the autopilot and will not succeed
            on rover type vehicles in simulation.
        """
        self._vehicle.groundspeed = velocity
    
    async def _stop(self):
        """
        Internal utility to stop any movement being run in the background. This
        ignores await_ready_to_move(). Requires set_velocity to be implemented
        for the given vehicle.

        TODO needs testing. setting velocity to 0\bar may not be best way to
        stop
        """
        self._ready_to_move = lambda _: True

class Drone(Vehicle):
    """
    Drone vehicle type. Implements all functionality that AERPAW's drones
    expose to user scripts, which includes basic movement control (going to
    coords, turning, landing).
    """
    async def set_heading(self, heading: float, blocking: bool=True, lock_in: bool=True):
        """
        Set the heading of the vehicle (in absolute deg).
        
        To turn a relative # of degrees, you can do something like
        `drone.set_heading(drone.heading + x)`

        Pass in `None` to make the drone return to ardupilot's default heading
        behavior (usually facing the direction of movement)

        If `blocking` is `True`, this will wait for the current movement command
        to finish before setting the controller's yaw -- in this case, the drone
        will immediately start turning. If `blocking` is `False`, the new yaw 
        will only affect future commands that interact with the yaw controller
        (ex: `set_velocity`)

        If `lock_in` is `True`, subsequent commands that could affect the heading
        will respect the heading set by this command. If it is false, consider this
        call to temporarily set the heading.

        An example of this in action is as follows: set_heading is called to lock in
        a 90 degree heading, followed by a goto command. The goto command will look
        at the 90 degrees heading set before. If lock_in were false, the goto
        command would make the drone face the direction of travel.
        """
        if blocking:
            await self.await_ready_to_move()

        if heading == None:
            self._current_heading = None
            return

        heading %= 360
        if lock_in:
            self._current_heading = heading
        if not blocking:
            return
        
        msg = self._vehicle.message_factory.command_long_encode(
            0, 0,                                       # target system, component
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
        await asyncio.sleep(5)

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

    async def goto_coordinates(self,
            coordinates: util.Coordinate,
            tolerance: float=2,
            target_heading: float=None):
        if target_heading != None:
            await self.set_heading(target_heading)
        
        await self.await_ready_to_move()
        await self._stop()

        heading = float('nan')
        if self._current_heading != None:
            heading = self._current_heading
        else:
            heading = self.position.bearing(coordinates)
        await self.set_heading(heading, lock_in=False)

        self._vehicle.message_factory.mission_item_send(
            0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            2, 0, # unused
            0, # hold time
            0, # accept radius
            0, # pass radius
            heading, # yaw
            coordinates.lat, coordinates.lon, coordinates.alt
        )
        
        # TODO in the future we likely want to split alt into a different tolerance
        at_coords = lambda self: \
            coordinates.distance(self.position) <= tolerance
        self._ready_to_move = at_coords

        while not at_coords(self): await asyncio.sleep(_POLLING_DELAY)

    _velocity_loop_active: bool=False

    async def set_velocity(self,
            velocity_vector: util.VectorNED,
            global_relative: bool=True,
            duration: float=None):
        await self.await_ready_to_move()
        
        self._velocity_loop_active = False # TODO race condition bleh
        await asyncio.sleep(_POLLING_DELAY)
        
        if not global_relative:
            velocity_vector = velocity_vector.rotate_by_angle(-self.heading)

        bitmask = 0b0000111111000111     # bitmask to only set speed
        yaw = 0
        if self._current_heading is not None:
            bitmask &= _YAW_BITMASK
            yaw = self._current_heading

        msg = self._vehicle.message_factory.set_position_target_global_int_encode(
                0, 0, 0,                # unused, target sys, component
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                bitmask,
                0, 0, 0,                # unused
                velocity_vector.north,
                velocity_vector.east,
                velocity_vector.down,
                0, 0, 0,                # (unsupported) accel
                math.radians(yaw), 0    # yaw/rate
                )
        
        self._ready_to_move = lambda _: True
        target_end = time.time() + duration if duration is not None else None
        
        async def _velocity_helper():
            while self._velocity_loop_active:
                if target_end is not None and time.time() > target_end:
                    self._velocity_loop_active = False
                self._vehicle.send_mavlink(msg)
                await asyncio.sleep(0.1)        # TODO tune for better perf
        self._velocity_loop_active = True
        asyncio.ensure_future(_velocity_helper())

    async def _stop(self):
        await super()._stop()
        if self.armed:
            await self.set_velocity(util.VectorNED(0, 0, 0))
        self._velocity_loop_active = False

class Rover(Vehicle):
    """
    Rover vehicle type. Implements all functionality that AERPAW's rovers
    expose to user scripts, which includes basic movement control (going to
    coords).

    `target_heading` is ignored for rovers, as they can't strafe.
    """
    async def goto_coordinates(self,
            coordinates: util.Coordinate,
            tolerance: float=2,
            target_heading: float=None):
        await self.await_ready_to_move()
        self._vehicle.simple_goto(util.Coordinate(coordinates.lat, coordinates.lon, 0).location())
        
        at_coords = lambda self: \
            coordinates.ground_distance(self.position) <= tolerance
        self._ready_to_move = at_coords

        while not at_coords(self): await asyncio.sleep(_POLLING_DELAY)

# TODO break this down further?:
# class LAM(Drone)
# class SAM(Drone)
