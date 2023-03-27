# zmq_preplanned_orbit

This is an example aerpaw mission that uses 2 drones to demonstrate how to use the aerpaw multi-vehicle control backend.

The mission flown involves two drones, one of which is a "tracer" and the other is an "orbiter". A ground coordinator is also used to orchestrate the movement.

The tracer will fly between several waypoints defined by a .plan file (if following instructions below, orbit.plan).
The orbiter will fly with the tracer between waypoints and orbit the tracer at each waypoint.
The ground coordinator keeps the mission in sync and commands the drones.

Each drone has a script, however the pattern proposed by this example leaves minimal code on the drones.
Most logic -- and more specifically all mission structure -- is defined within the ground coordinator.

## running this example

This example can be run using any experiment with two drones and a cloud node.
Alternatively, the cloud node software can be run on a drone, however that will require additional undocumented setup.

The software being run on each node is described below.
In the actual aerpaw testbed, the ip addresses will have to be changed.
These addresses are assuming that this is being run locally, with a mavproxy instance forwarding SITL to each of the referenced ports.

## Tracer

```bash
# zmq-identifier and zmq-proxy-server are required for zmq-based scripts
python -m aerpawlib --vehicle drone --conn 127.0.0.1:14570 \
    --zmq-identifier tracer --zmq-proxy-server 127.0.0.1 \
    --script drone_tracer
```

## Orbiter

```bash
# zmq-identifier and zmq-proxy-server are required for zmq-based scripts
python -m aerpawlib --vehicle drone --conn 127.0.0.1:14580 \
    --zmq-identifier orbiter --zmq-proxy-server 127.0.0.1 \
    --script drone_orbiter
```

## Ground

```bash
# launch the zmq broker/proxy
# this is done through an aerpawlib internal tool, which still (for now) requires
# specifying what script/connection to use. the values provided do not matter
python -m aerpawlib --run-proxy

# launch ground coordination software
# params are documented in help function
# zmq-identifier and zmq-proxy-server are required for zmq-based scripts
python -m aerpawlib --vehicle none --conn a --skip-init \
    --zmq-identifier ground --zmq-proxy-server 127.0.0.1 \
    --script ground_coordinator --file orbit.plan 
```
