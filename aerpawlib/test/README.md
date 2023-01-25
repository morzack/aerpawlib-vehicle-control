# aerpawlib tests

This is really just a place to throw some tests and give myself a bit of extra sanity when writing weird code.
Most things that go in this library probably can't be easily tested w/out writing some kind of mocking library, so why bother.

This is a good place for future work to be done, especially if we end up releasing the lib to the public or anything like that.

## Running tests

Tests are built using `pytest`

```
aerpawlib/test >> pip install -U pytest

aerpawlib/test >> pytest
```


## Manual Simulation Tests
In addition to unit testing the util.py API, this folder contains a variety of parameter files to run the speed test and rover search to validate the behavior of the safety checker in util.py

### Speed test script tests
Here is a step by step guide to running the speed test
1. Make sure your current directory is `/AHN/E-VM/Profile_software/vehicle_control/aerpawlib/aerpawlib/test`
2. Determine which geofence config file and speed test config file you want to use
3. Once you know which params files the simulation test can be run as `python3 run_safety_checker_sitl_tests.py --script_params speed_test_params/valid_1.yaml --geofence_params geofence_config_copter_test.yaml`
4. You will need to manually arm and takeoff the copter from the mavproxy screen. Copy the launched service that resembles `screen -R a6e6_mavproxy` and run it in a separate terminal window.
5. Similarly copy the launched service for the vehicle script `screen -R a6e6_vehicle_script` and run it in another terminal window
6. Once the mavproxy terminal reads, 
    ```
    AP: EKF3 IMU1 is using GPS
    AP: EKF3 IMU0 is using GPS
    ```
    Run the following commands
    1. `mode guided`
    2. `arm throttle`
    3. `takeoff 30` or whatever altitude (in meters) is desired
 7. After the copter has reached altitude, in the vehicle script terminal press 'Enter' to start the speed test
 8. Monitor the copter's progress from QGround Control with a comm UDP link on port 14570
 9. At each waypoint the copter will adjust its heading to face the next waypoint and wait for a desired speed to be input in the vehicle script terminal.