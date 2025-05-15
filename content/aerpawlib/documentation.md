`aerpawlib` is a set of frameworks and tools that abstract away lower level
concepts needed to write complex scripts that run on the AERPAW platform. This
lib is intended to be used by both internal developers and researchers.

aerpawlib has two main design goals:

- Users of this lib must be able to directly translate mission logic into code
    without worrying about higher level programming concepts.
- The library must remain as Pythonic as possible, both internally and with how
    it exposes itself.

## Getting Started with aerpawlib

This guide will take you through all the steps needed to get a basic
aerpawlib driven program working from scratch. It will also explain some of
the design decisions made by aerpawlib in some depth.

### Installing aerpawlib

Before installing aerpawlib, there are a few dependencies that you need to
grab:

- `python` -- versions between `3.6` and `3.9` are supported
- `pip` -- should be bundled with python in most distributions

To install aerpawlib:

- Clone this repo. Avoid changing the path if possible after installation.
- `cd AERPAW-Dev/AHN/E-VM/Profile_software/vehicle_control/aerpawlib`
- Install as a local lib -- `pip install --user -e .`

After following these steps, aerpawlib will now be usable in any python
script, provided that the path isn't changed.

To update aerpawlib, you can simply `git pull` inside the dir to sync with
the upstream repo.

### Creating a basic runner

aerpawlib is built so that any script made with it in mind can be run in the
same way. To do this, it sacrifices a bit of functionality and enforces that
scripts implement a `aerpawlib.runner.Runner`.

There are two main runners used by aerpawlib -- the
`aerpawlib.runner.BasicRunner`, and the `aerpawlib.runner.StateMachine`:

- `aerpawlib.runner.BasicRunner` -- has a single entry point, all flow
    control is handled by the user program
- `aerpawlib.runner.StateMachine` -- has multiple states of various types,
    flow between states is handled by aerpawlib

`aerpawlib.runner.BasicRunner` is appropriate for the most basic of scripts,
so we'll use that here. It can be imported directly and linked to your script's
`aerpawlib.runner.Runner` instance by doing the following:

```python
from aerpawlib.runner import BasicRunner
from aerpawlib.vehicle import Drone

class MyScript(BasicRunner):
    # more things...
```

When using a `aerpawlib.runner.BasicRunner`, you're allowed to create a single
function that serves as an entry point for your code. This function -- decorated
by `aerpawlib.runner.entrypoint` -- will be called and passed a `Vehicle` at
runtime that you can interact with.

```python
    # more things...
    @entrypoint
    async def my_function(self, vehicle: Drone):
        print("I'm doing things!")
```

[note: don't worry about the `async` yet! that'll be talked about a bit later]

Now you can run your script using the aerpawlib CLI tool:

```bash
python -m aerpawlib \
    --script path/to/your/script \  # without the .py!
    --conn udp:127.0.0.1:14550 \    # or whatever SITL is using
    --vehicle drone
```

```txt
# output after connecting:
I'm doing things!
```

The above command assumes you have SITL installed and it's up and running. To
install SITL, check out [TODO]().

### Interacting with vehicles

As mentioned before, the function marked by `aerpawlib.runner.entrypoint` in
your script will always be passed a `aerpawlib.vehicle.Vehicle` of some kind
-- this vehicle is determined by the vehicle *type* passed to the command line
tool.

There are a few kinds of vehicles that you can work with, each under
`aerpawlib.vehicle`. For this script, we'll be using a `Drone` since it has
more capabilities.

The aerpawlib CLI tool will fully initialize whatever vehicle you're using to
be ready to use, however you must explicitly make it take off at the start of
your script.

```python
async def my_function(self, vehicle: Drone):
    # ...
    await vehicle.takeoff(5) # take off to 5 meters
```

Once the drone is in the air, you have full control over its position and
heading by using `aerpawlib.vehicle.Vehicle.goto_coordinates` and
`aerpawlib.vehicle.Drone.set_heading`.

Additionally, `aerpawlib.vehicle.Vehicle`s expose properties related to their
current physical state, such as their battery level and current position.

### Using coordinates

aerpawlib is designed to *only* have an understanding of absolute coordinates
using latitude and longitude -- this removes an entire class of bugs and
confusion related to whether or not a vehicle is moving relative to its start
position or world-scale coordinates. Imagine if you tried to send a vehicle to
`(0, 0)`, but instead of coming home, it tried to fly out into the middle of
the ocean!

Keeping this in mind, aerpawlib wraps all positions inside the
`aerpawlib.util.Coordinate` object. This is a representation of an absolute
coordinate, with the altitude being relative to where the vehicle started.

To transform `aerpawlib.util.Coordinate`s, there's also a related
`aerpawlib.util.VectorNED` class -- this is a 3 dimensional vector in the
[NED (north, east, down)](https://en.wikipedia.org/wiki/Local_tangent_plane_coordinates#Local_north,_east,_down_(NED)_coordinates)
schema. aerpawlib has overridden some of Python's operators so that you can
cleanly combine coordinates with vectors.

```python
from aerpawlib.util import Coordinate, VectorNED

x = Coordinate(35.771634, -78.674109)

# get a new coord 10m north and 5m west of X
y = x + VectorNED(10, -5, 0)

# find the bearing (heading difference) between X and Y
x.bearing(y)    # -> 315 degrees

# get the vector between Y and X
vec = y - x     # -> VectorNED(-10, 5, 0)
```

### Moving a vehicle

So by using a vehicle's current position combined with a vector representing
how you want to translate it, you have nearly full control over a vehicle's
position.

As an example, let's move a drone in a square pattern:

```python
async def my_function(self, vehicle: Drone):
    # ...
    # go north
    await vehicle.goto_coordinates(vehicle.position + VectorNED(10, 0, 0))
    await asyncio.sleep(5) # seconds
    
    # go west
    await vehicle.goto_coordinates(vehicle.position + VectorNED(0, -10, 0))
    await asyncio.sleep(5)
    
    # go south
    await vehicle.goto_coordinates(vehicle.position + VectorNED(-10, 0, 0))
    await asyncio.sleep(5)
    
    # go east
    await vehicle.goto_coordinates(vehicle.position + VectorNED(0, 10, 0))
    await asyncio.sleep(5)

    # don't forget to land when you're done!
    await vehicle.land()
```

### Asynchronous programming

Because the physical state of a drone and the state of a program aren't
inherently linked, aerpawlib uses asyncio internally to help with syncing
things up.

What this means for you, the developer of a script, is that there are two
actions that your script can take after telling a drone what to do -- it can
either wait for the drone to complete the action before resuming the flow, or
it can continue running while waiting for the drone to finish moving in the
background.

In Python, these two actions are specified using the `await` keyword, or the
function `asyncio.ensure_future`:

```python
async def my_function(self, vehicle: Drone):
    # go north and (a)wait for the drone to arrive
    await vehicle.goto_coordinates(vehicle.position + VectorNED(10, 0, 0))

    # go south and immediately print some things out
    action = asyncio.ensure_future(vehicle.goto_coordinates(vehicle.position
                                                            + VectorNED(0, 10, 0)))
    print("going south now...")

    # wait for the action to be finished
    await action
```

In the second example above, we capture the `asyncio.Task` returned by
`asyncio.ensure_future`, which allows for us to then `await` it later on. It's
not necessary to do this unless you want to use `await` later.

**Important!**

One caveat of using asyncio is that you can't use any functions that would
normally block the execution of the program. Fortunately, asyncio provides
implementations of the common culprits, such as `time.sleep` -- you can see how
they're used in some of the above examples.

### Background tasks

One of the benefits of asyncio is that it allows for aerpawlib to cleanly
implement tasks that run in the background. aerpawlib exposes this
functionality through the `aerpawlib.runner.background()` decorator, which
causes the decorated function to be repeatedly called in the background:

```python
class MyRunner(StateMachine):
    @background
    async def log_in_background(self, vehicle: Vehicle):
        print(f"vehicle pos: {vehicle.position}")
        await asyncio.sleep(1)
```

Note that background tasks are only available to runners that use the
`aerpawlib.runner.StateMachine` framework.

### Creating a state machine

The more powerful of aerpawlib's two frameworks is
`aerpawlib.runner.StateMachine`: this framework lets you create a runner with
multiple "states" that can be transitioned between by the runner.

A state is just a function decorated by the `aerpawlib.runner.state` decorator.
This decorator allows for a state to be given a name -- this name is what
aerpawlib uses to identify states when transitioning between them.

When the state function is done running, it should return a string that is the
next state for the runner to run. If no string is returned, aerpawlib assumes
that the script is done running.

The first state used in a script must be designated by setting the
`first` parameter to true when creating it.

Finally, there's also a `aerpawlib.runner.timed_state()` decorator available.
This decorator will make sure that a state is run for *at least* a set number
of seconds before transitioning to the next state, even if the state exits
early. If the function designated as a state takes longer than the allotted
time, it will continue running until a value is returned or the function
exits.

Here's an example of a state machine that makes use of all the things we've
talked about so far. It makes a drone continually "take measurements" spaced
10m apart until it gets a measurement that is 0, at which point it lands.

```python
class MyScript(StateMachine):
    @state(name="start", first=True)
    async def start(self, vehicle: Vehicle):
        await vehicle.take_off(5)
        return "go_north"

    @state(name="go_north")
    async def go_north(self, vehicle: Vehicle):
        # go north and continually log the vehicle's position
        moving = asyncio.ensure_future(vehicle.goto_coordinates(vehicle.position + VectorNED(10, 0, 0)))

        while not moving.done():
            print(f"pos: {vehicle.position}")
            await asyncio.sleep(0.1)

        await moving
        return "take_measurement"

    @timed_state(name="take_measurement", duration=5)
    async def take_measurement(self, vehicle: Vehicle):
        # take a measurement and wait for 5 seconds
        measurement = random.randint(0, 5)

        if measurement == 0:
            return "land"
        else:
            return "go_north"

    @state(name="land")
    asnyc def land(self, vehicle: Vehicle):
        await vehicle.land()
```

## Advanced aerpawlib: Multi-vehicle control

This is a continuation of the previous guide that will go over how to structure and write advanced aerpawlib scripts that control multiple vehicles.
There is an expectation that the reader is familiar with aerpawlib, but no knowledge of the underlying stack (zmq) is needed.

As aerpawlib's primary objective is to make it easy to translation mission logic into code, being able to support multi-vehicle experiments was a natural extension of its capabilities.
This overarching goal was one of the driving factors behind the usage of state machines and async code.

Notably, aerpawlib's multi-vehicle control architecture was designed to give as much flexibility to a user as possible, however it remains opinionated to a degree.
Multi-vehicle control software is hard, there are many ways to implement it, and providing a single standard helps avoid unmaintainable logic (ultimately giving safer code :) ).

### The Multi-Vehicle Backend

aerpawlib's multi-vehicle control is built around a zmq pub/sub stack, which connects all aerpawlib scripts through a central proxy/"broker".
This broker is a built in utility in aerpawlib, and can be run through the aerpawlib module.
Each aerpawlib script must connect back to the broker and identify itself with an identifier -- these identifiers are provided by the user at runtime, but should be known in advance to make writing code possible.

There are two main exposed pieces of functionality within the aerpawlib multi-vehicle framework: transitioning the state of a runner, and querying some value from a runner.
In the backend, these are implemented by routing requests to handler functions declared by the zmq runner, which in turn runs through each request as it comes in in a separate async function.

### The Multi-Vehicle Structure

As previously mentioned, aerpawlib is opnionated and designed around a set of design points.
For multi-vehicle scripts, this is even more evident.
It's difficult to explain the tenets in whole (examples are below), but here are the key points to keep in mind when designing aerpawlib scripts:

1. aerpawlib scripts run on drones connected through mavproxy should implement *low-level commands* (ex: fly north 10m, return to launch)
2. there should be one aerpawlib script not connected physically to a drone through mavproxy that implements *high-level experiment logic* (ex: send two drones to an area, tell drones to perform a grid scan)

By decoupling experiment logic from drone control as much as possible, an experiment can keep its logic entirely within a singular state machine, only using the state machines on the vehicles to control the movement at a low level.

### A basic multi-vehicle script

With this knowledge in hand, we can now start to build a multi-vehicle experiment.
This experiment is absed off the zmq_preplanned_orbit example.
For the purposes of this demo, we want to fly two drones through a series of waypoints (determined by a .plan file), and have one drone "orbit" the other drone at each waypoint.
The drones should remain synchronized with each other throughout the experiment.

To make this happen, we need 3 aerpawlib scripts:

- ground_coordinator -- this will read from the .plan file and send drones to coordinates
- drone_tracer -- this will implement commands for the tracer drone, sending it from point to point
- drone_orbiter -- this will implement commands for the orbiter drone, making it follow the tracer and orbit at certain points

For the sake of brevity in this documentation page, all code is contained within the `examples/zmq_preplanned_orbit` folder.
This will only pull snippets that implement specific patterns.

### Pattern 1: Vehicle Control

As mentioned previously, scripts running on a vehicle should only implement low-level mission commands.
Let's look at a vehicle state machine to see how that's done.

`drone_tracer.py`

```python
class TracerRunner(ZmqStateMachine):
    # ...
    @state(name="wait_loop", first=True)
    async def state_wait_loop(self, _):
        await asyncio.sleep(0.1)
        return "wait_loop"

    @state(name="next_waypoint")
    async def state_next_waypoint(self, drone: Drone):
        coords = await self.query_field(ZMQ_GROUND, "tracer_next_waypoint")
        await drone.goto_coordinates(coords)
        await self.transition_runner(ZMQ_GROUND, "callback_tracer_at_waypoint")
        return "wait_loop"
    
    # ...
```

Fundementally, there is always one resting state for the vehicle -- `wait_loop`.
This state will continually idle the state machine, not sending any commands to the underlying hardware.
Other states such as `next_waypoint` are used to control the drone itself, in this case it gets a position managed by the ground station, sends the drone to it, and then reports back the completation of the task before returning to an idle state.

Now, let's see what the ground control station is doing:

`ground_coordinator.py`

```python
class GroundCoordinatorRunner(ZmqStateMachine):
    # ...
    @state(name="next_waypoint")
    async def state_next_waypoint(self, _):
        # ...
        await self.transition_runner(ZMQ_TRACER, "next_waypoint"),
        await self.transition_runner(ZMQ_ORBITER, "next_waypoint"),

        self._tracer_at_waypoint = False
        self._orbiter_at_waypoint = False
        
        return "await_in_transit"

    # funcs to calculate where each drone should go
    @expose_field_zmq(name="tracer_next_waypoint")
    async def get_tracer_next_waypoint(self, _):
        waypoint = self._waypoints[self._current_waypoint]
        coords = Coordinate(*waypoint["pos"])
        return coords
    
    # ...
    
    @state(name="await_in_transit")
    async def state_await_in_transit(self, _):
        if not (self._tracer_at_waypoint and self._orbiter_at_waypoint):
            return "await_in_transit"
        return "orbiter_start_orbit"

    @state(name="callback_tracer_at_waypoint")
    async def callback_tracer_at_waypoint(self, _):
        self._tracer_at_waypoint = True
        return "await_in_transit"
    
    # ...
```

The entry point for this set of states is `next_waypoint`.
Let's run through the flow:

1. First, the ground station tells both drones to activate the `next_waypoint` state, which is the command discussed before.
2. Then, the ground station sets two flags referencing command completion to false. These will be updated by the drones as they finish their commands.
3. At this point the drones have begun their commands, and the ground station is now idling. When both flags are set to true by the drones, the script will continue.
4. Some drone will now finish the command. It calls the callback (ex: `callback_tracer_at_waypoint`), setting the flag for that drone and immediately returning to the waiting loop.
5. Finally, when both drones are done/both flags are set, the script can continue to the next state

Syntactically, this is an ugly implementation, however it is clean and straightforwards.
By centralizing flow, extremely complicated scripts can be developed that dodge potential spaghetti logic.

### Pattern 2: Querying fields

Querying a field or value from another vehicle/script in aerpawlib is simple, you simply have to call the `query_field` function.
This function will request the field from the relevant drone, blocking until it's received.
An example is below:

`drone_orbiter.py`

```python
class OrbiterRunner(ZmqStateMachine):
    @expose_field_zmq(name="position")
    async def get_drone_position(self, drone: Drone):
        return drone.position
    # ...
```

`ground_coordinator.py`

```python
class GroundCoordinatorRunner(ZmqStateMachine):
    # ...
    @expose_field_zmq(name="orbiter_next_waypoint")
    async def get_orbiter_next_waypoint(self, _):
        waypoint = self._waypoints[self._current_waypoint]
        coords = Coordinate(*waypoint["pos"])
        # note the query_field being awaited
        tracer_pos = await self.query_field(ZMQ_TRACER, "position")
        tracer_delta = coords - tracer_pos
        # note the query_field being awaited, again
        orbiter_pos = await self.query_field(ZMQ_ORBITER, "position")
        orbiter_next_pos = orbiter_pos + tracer_delta
        return orbiter_next_pos
    # ...
```

## aerpawlib Vehicle Configuration

aerpawlib provides support for adding constraints to vehicles that are checked during runtime.
The purpose of this is to enable safer characterisation of vehicle behaviour before enabling more dangerous features.
When launching the aerpawlib runner, a custom vehicle configuration file can be provided by using the `--vehicle-config` command line argument:

```bash
python -m aerpawlib \
    --script examples/squareoff_logging \
    --conn udp:127.0.0.1:14550 \
    --vehicle drone \
    --vehicle-config vehicle_configurations/AERPAW_SAM.yaml
```

There are several example configurations available in the vehicle_configurations folder for existing AERPAW based drones.
`vehicle_config_exhaustive.yaml` serves as documentation for the various fields, with comments explaining functionality.
