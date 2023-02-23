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

### run_safety_checker_sitl_tests.py
The script `run_safety_checker_sitl_tests.py` provides a convenient way to launch the SITL, mavproxy, filter, and optionally a vehicle script automatically. By default the script anticipates to launch the vehicle speed test and requires the parameter `--script_params <speed_test_params_file.yaml>`. However instead you can pass in the parameter `--no_script True` to bypass this and only launch SITL, mavproxy, and the filter. In this case, the script will output a connection address for you to use when manually launching an aerpawlib vehicle script.

### Speed test script tests
Here is a step by step guide to running the speed test
1. Make sure your current directory is `/AHN/E-VM/Profile_software/vehicle_control/aerpawlib/aerpawlib/test`
2. Determine which geofence config file and speed test config file you want to use
3. Once you know which params files the simulation test can be run as `python3 run_safety_checker_sitl_tests.py --script_params speed_test_params/valid_1.yaml --geofence_params geofence_config_copter_test.yaml` with the yaml files replaced with whatever is desired 
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

 #### Test variations
The following are all of the designed combinations of vehicle config and speed test params to test different aspects of the safety checker and the overall system

*Note: In all of the following combinations, you can replace `geofence_config_copter_test.yaml` with `geofence_config_rover_test.yaml` to test the behavior of a rover instead of a copter*

 1. `geofence_config_copter_test.yaml` + `valid_1.yaml`
    1. This combination should run the speed test with two valid locations and shouldn't error
 2. `geofence_config_copter_test.yaml` + `out_of_fence_start.yaml`
    1. This combination gives an invalid first waypoint that should trigger the filter to error
    2. `Invalid waypoint. Waypoint (35.728572845458984,-78.69427490234375) is outside of the geofence. ABORTING!`
 3. `geofence_config_copter_test.yaml` + `invalid_second_waypoint.yaml`
    1. This combination is identical to the one above, except the first location is valid and the script should error once it attempts to move towards the second waypoint.
    2. `Invalid waypoint. Waypoint (35.728572845458984,-78.69427490234375) is outside of the geofence. ABORTING!`
 4. `geofence_config_copter_test.yaml` + `valid_2.yaml`
    1. This is a combination that shouldn't error. The speed test contains 4 waypoints that circle the animal health building which is excluded from the geofence.
 5. `geofence_config_copter_test.yaml` + `invalid_leave_fence.yaml`
    1. This combination contains a speed test with two waypoints that appear to be valid. However to move between them would result in crossing out of the geofence.
    2. `Invalid waypoint. Path from (35.7274213,-78.6965512) to waypoint (35.72699737548828,-78.69512176513672) leaves geofence. ABORTING!`
 6. `geofence_config_copter_test.yaml` + `invalid_enter_no_go.yaml`
    1. This combination gives a valid first waypoint but a second waypoint contained within the nogo zone.
    2. `Invalid waypoint. Waypoint (35.72568893432617,-78.69639587402344) is inside a no-go zone. ABORTING!`
 7. `geofence_config_copter_test.yaml` + `invalid_cross_no_go.yaml`
    1. This combination contains two waypoints that appear to be valid, but to move between them would result in crossing a no go zone.
    2. `Invalid waypoint. Path from (35.7274223,-78.6965523) to waypoint (35.723670959472656,-78.69671630859375) enters no-go zone. ABORTING!`