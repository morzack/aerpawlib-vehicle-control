# zmq_preplanned_orbit

This is an example aerpaw mission that uses 2 drones to demonstrate how to use the aerpaw multi-vehicle control backend.

The mission flown involves two drones, one of which is a "tracer" and the other is an "orbiter". A ground coordinator is also used to orchestrate the movement.

The tracer will fly between several waypoints defined by a .plan file (if following instructions below, orbit.plan).
The orbiter will fly with the tracer between waypoints and orbit the tracer at each waypoint.
The ground coordinator keeps the mission in sync and commands the drones.

Each drone has a script, however the pattern proposed by this example leaves minimal code on the drones.
Most logic -- and more specifically all mission structure -- is defined within the ground coordinator.

## running this example
