# MODIFIED FROM: John Kesler's run_sitl.py

from argparse import ArgumentParser
import uuid
import os

BASE_SYS_ID = 51
BASE_INSTANCE = 1
BASE_LOCATION = "aerpaw2"


# all relative paths from E-VM/Profile_software/vehicle_control/aerpawlib/aerpawlib/test
FILTER_EXEC = "../../../../../../C-VM/MAVLink_Filter/filter.py"
FILTER_ALLOWED_COPTER = "../../../../../../C-VM/MAVLink_Filter/allowed-copter.json"
FILTER_ALLOWED_ROVER = "../../../../../../C-VM/MAVLink_Filter/allowed-rover.json"
SITL_BASE_ARGS = "-w -v ArduCopter --no-mavproxy"

# modify path to local install of ardupilot
SITL_EXEC = "../../../../../../../..//ardupilot/Tools/autotest/sim_vehicle.py"

SPEED_TEST_DIR = "../../../SpeedTest/"
SPEED_TEST = "speed_test"

# screen prefix names
SCREEN_SITL_PREFIX = "sitl"
SCREEN_MAV_PREFIX = "mavproxy"
SCREEN_FILTER_PREFIX = "filter"
SCREEN_VEHICLE_SCRIPT_PREFIX = "vehicle_script"


if __name__ == "__main__":
    parser = ArgumentParser(
        description="run-safety-checker-sitl-tests - trigger tooling to spawn sitl and a vehicle script under screen"
    )
    parser.add_argument(
        "--sitl-instance", required=False, default="1", dest="sitl_instance"
    )
    parser.add_argument("--script_params", required=True, dest="script_params")
    parser.add_argument("--geofence_params", required=True, dest="geofence_params")
    args, _ = parser.parse_known_args()

    sitl_instance = 1

    sys_id = BASE_SYS_ID
    sys_instance = BASE_INSTANCE

    # Takeoff location
    location_raw = "35.72731601510149,-78.69644259405045,0,270"
    sitl_cmd = f"{SITL_EXEC} {SITL_BASE_ARGS} -l {location_raw} --no-rebuild"

    mav_sitl_port = 5760
    mav_output_base = 14570

    mav_cmd = "mavproxy.py --master=tcp:127.0.0.1:5760 --out udp:127.0.0.1:14570 --out udp:127.0.0.1:14571 --out udp:127.0.0.1:14573"

    # between user entrypoint and mavproxy

    filter_cmd = f"python3 {FILTER_EXEC} --downlink 127.0.0.1:14575 --port 14573 --vehicle_config {args.geofence_params} --allowed_messages {FILTER_ALLOWED_COPTER}"

    # we cd to run the script so we must modify the local script_params path
    script_params_file = os.path.join(os.getcwd(), args.script_params)
    script_cmd = f"cd {SPEED_TEST_DIR} && pwd && python -m aerpawlib --script {SPEED_TEST} --conn udp:127.0.0.1:14575 --vehicle drone --params {script_params_file} --skipoutput"

    print(sitl_cmd)
    print(mav_cmd)
    print(filter_cmd)

    screen_prefix = str(uuid.uuid4())[:4]

    # Launch SITL
    # add -L to a screen command to log to file
    os.system(f"screen -S {screen_prefix}_{SCREEN_SITL_PREFIX} -dm {sitl_cmd}")
    # print(mav_cmd_with_filter)
    os.system(f"screen -S {screen_prefix}_{SCREEN_MAV_PREFIX} -dm {mav_cmd}")
    os.system(f"screen -L -Logfile screenlog_filter -S {screen_prefix}_{SCREEN_FILTER_PREFIX} -dm {filter_cmd}")
    os.system(
        f"screen -L -Logfile screenlog_vehiclescript -S {screen_prefix}_{SCREEN_VEHICLE_SCRIPT_PREFIX} -dm bash -c '{script_cmd}'"
    )

    print("LAUNCHED SERVICES ON:")
    launched_services = [
        SCREEN_SITL_PREFIX,
        SCREEN_MAV_PREFIX,
        SCREEN_FILTER_PREFIX,
        SCREEN_VEHICLE_SCRIPT_PREFIX,
    ]
    for service in launched_services:
        print(f"screen -R {screen_prefix}_{service}")

    print("")

    exit_commands = [
        f"screen -S {screen_prefix}_{service} -X quit" for service in launched_services
    ]

    print("press enter to quit:")
    input()
    for cmd in exit_commands:
        os.system(cmd)
