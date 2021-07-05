`aerpawlib` is a set of frameworks and tools that abstract away lower level
concepts needed to write complex scripts that run on the AERPAW platform. This
lib is intended to be used by both internal developers and researchers.

aerpawlib is an opinionated library -- it has been designed while keeping a
"best" way of doing things in mind. However, while not encouraged, it's still
possible to go against some of those design decisions by peeking into some of
the features hidden within the lib.

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

```
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
