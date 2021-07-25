# aerpawlib Starter Profiles

## Squareoff with Logging

`squareoff_logging.py` is an example aerpawlib script that will fly a drone in
a 10m by 10m square, while using the `StateMachine`'s `background` utility to
continually log the drone's position to a file. This example is intended to
demonstrate how to write a dynamic state machine as well as use the
`background` tool.

This script can be run on either a drone or a rover. To run it, use aerpawlib:

```
python -m aerpawlib --conn ... --script squareoff_logging --vehicle drone
```

When run, it will continually output positional data to a log file that can be
specified using the `--output` param. The sample rate can be changed using
`--samplerate`. If no file is specified, it will default to outputting to a
file named `GPS_DATA_YYYY-MM-DD_HH:MM:SS.csv`. This output format is below:

```
line num,lon,lat,alt,battery voltage,timestamp,GPS fix type,# visible sats

timestamp format: YYYY-MM-DD HH:MM:SS.ssssss
```

A flowchart of the various states is below:

```
┌───────┐drone ┌──────────┐
│ start ├──────► take_off │
└───┬───┘      └─────┬────┘
    │                │
    ├────────────────┘
    │
┌───▼───────┐
│ leg_north ├───────────┐
│           │           │
│ leg_west  │           │
│           │       ┌───▼─────────┐
│ leg_south ◄───────┤ at_position │
│           │pick   └───┬──┬────▲─┘
│ leg_east  │based on   │  └────┘sleep 5s
└───────────┘current_leg│
                    ┌───▼────┐drone ┌──────┐
                    │ finish ├──────► land │
                    └────────┘      └──────┘
```

The centerpiece of this script, aside from the background logging, is the
dynamic state changing. By default, the only state stored by a `StateMachine`
is the current state *function* -- individually, each function can be
considered to be stateless (with side effects affecting the vehicle).

To introduce additional state into the state machine, all you have to do is add
method variables to your `StateMachine` derived object. This script uses
`_legs: List[str]` and `_current_leg: int` to do that. `_current_leg` is
altered and interpreted by the `at_position` state to then dynamically pick the
next state on the fly.

## Preplanned Trajectory

`preplanned_trajectory.py` is an example aerpawlib script that makes a vehicle
fly between different waypoints read from a `.plan` file generated using
`QGroundControl`. This example is a good starting point for experiments that
make use of non-dynamic flight plans.

This script can be run on either a drone or rover. To run it, use aerpawlib:

```
python -m aerpawlib --conn ... --script preplanned_trajectory --vehicle drone \
    --file <.plan file to use>
```

When run, it will load in the `.plan` file located at the path specified by
`--plan` and then send the drone to each waypoint specified sequentially in it.

A flowchart of the various states is below:

```
┌──────────┐
│ take_off │
└──────┬───┘
       │
       ├────────────────────────────────────────┐
       │                                        │
┌──────▼────────┐    ┌────────────┐     ┌───────┴─────┐
│ next_waypoint ├────► in_transit ├─────► at_waypoint │
└──────┬────────┘    └────────────┘     └─────────────┘
       │
    ┌──▼──┐
    │ rtl │
    └─────┘
```

This script includes several states that can be used as hooks to introduce
custom logic that runs during various parts of the flight plan.

`in_transit` is a function that will be called once after the script picks a
waypoint for the vehicle to go to, at which point it blocks until the vehicle
arrives. Custom logic can be added before the `await` statement, at which point
the script blocks while waiting for the drone to finish moving.

To add custom logic that waits for the drone to finish moving, you can
continually poll `drone.done_moving()`.

`at_waypoint` is a timed state that is called once the vehicle arrives at a
waypoint. As a timed state, it is guaranteed to be called repeatedly for at
least `duration` seconds.
