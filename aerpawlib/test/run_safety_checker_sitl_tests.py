# MODIFIED FROM: John Kesler's run_sitl.py
# intended to run from E-VM/Profile_software/vehicle_control/aerpawlib/aerpawlib/test

from argparse import ArgumentParser
import uuid
import os
import yaml

BASE_SYS_ID = 51
BASE_INSTANCE = 1

# modify path to local install of ardupilot
ARDUPILOT_BASE = "../../../../../../../../ardupilot"
# do not modify - relative path from ARDUPILOT_BASE
SITL_EXEC = "/Tools/autotest/sim_vehicle.py"

SITL_BASE_ARGS = "-w --no-mavproxy"
SITL_COPTER_ARGS = "-v ArduCopter"
SITL_ROVER_ARGS = "-v Rover"

# all relative paths from E-VM/Profile_software/vehicle_control/aerpawlib/aerpawlib/test
FILTER_BASE = "../../../../../../C-VM/MAVLink_Filter/"
# relative paths from FILTER_BASE
FILTER_EXEC = "filter_safety_checker.py"
FILTER_ALLOWED_COPTER = "allowed-copter.json"
FILTER_ALLOWED_ROVER = "allowed-rover.json"

VEHICLE_SCRIPT_PORT = 14575
# speed test vehicle script info
SPEED_TEST_DIR = "../../../../PortableNodeTests/SpeedTest"
SPEED_TEST = "speed_test"

SAFETY_CHECKER_SCRIPT = "../safetyChecker.py"
SAFETY_CHECKER_ADDR = "127.0.0.1"
SAFETY_CHECKER_PORT = 14580

# screen prefix names
SCREEN_PREFIX = "AERPAW"
SCREEN_SITL_PREFIX = "sitl"
SCREEN_MAV_PREFIX = "mavproxy"
SCREEN_FILTER_PREFIX = "filter"
SCREEN_SAFETY_CHECKER_PREFIX = "safety"
SCREEN_VEHICLE_SCRIPT_PREFIX = "vehicle_script"


if __name__ == "__main__":
    parser = ArgumentParser(
        description="run-safety-checker-sitl-tests - trigger tooling to spawn sitl and a vehicle script under screen"
    )
    parser.add_argument(
        "--sitl_instance", required=False, default="1", dest="sitl_instance"
    )
    parser.add_argument(
        "--script_params", required=False, default=False, dest="script_params"
    )
    parser.add_argument("--no_script", required=False, dest="no_script")
    parser.add_argument("--geofence_params", required=True, dest="geofence_params")
    args, _ = parser.parse_known_args()

    sitl_instance = 1

    sys_id = BASE_SYS_ID
    sys_instance = BASE_INSTANCE

    # Takeoff location
    location_raw = "35.72731601510149,-78.69644259405045,0,270"

    mav_sitl_port = 5760
    mav_output_base = 14570

    # setup script parameters based on vehicle type
    # read geofence params to determine vehicle type
    params_file = open(args.geofence_params, "r")
    params = yaml.load(params_file)
    vehicle_type = params["vehicle_type"]
    print(f"Vehicle type: {vehicle_type}")
    if vehicle_type == "copter":
        SITL_ARGS = f"{SITL_BASE_ARGS} {SITL_COPTER_ARGS}"
        SCRIPT_VEHICLE_TYPE = "drone"
        FILTER_ALLOWED_MSGS = f"{FILTER_BASE}{FILTER_ALLOWED_COPTER}"
        SITL_EXEC = f"{ARDUPILOT_BASE}{SITL_EXEC}"
    elif vehicle_type == "rover":
        SITL_ARGS = f"{SITL_BASE_ARGS} {SITL_ROVER_ARGS}"
        SCRIPT_VEHICLE_TYPE = "rover"
        FILTER_ALLOWED_MSGS = f"{FILTER_BASE}{FILTER_ALLOWED_ROVER}"
        SITL_EXEC = f"{ARDUPILOT_BASE}{SITL_EXEC}"
    else:
        print(
            f"VEHICLE TYPE {vehicle_type} IN {args.geofence_params} MUST BE EITHER copter OR rover!"
        )
    sitl_cmd = f"{SITL_EXEC} {SITL_ARGS} -l {location_raw} --no-rebuild"

    mav_cmd = "mavproxy.py --master=tcp:127.0.0.1:5760 --out udp:127.0.0.1:14570 --out udp:127.0.0.1:14571 --out udp:127.0.0.1:14573"

    filter_cmd = f"python3 {FILTER_BASE}{FILTER_EXEC} --downlink 127.0.0.1:{VEHICLE_SCRIPT_PORT} --port 14573 --allowed_messages {FILTER_ALLOWED_MSGS} --safety_checker_port {SAFETY_CHECKER_PORT} --safety_checker_addr {SAFETY_CHECKER_ADDR}"

    safety_checker_cmd = f"python3 {SAFETY_CHECKER_SCRIPT} --vehicle_config {args.geofence_params} --port {SAFETY_CHECKER_PORT}"

    print(f"SITL CMD: {sitl_cmd}")
    print(f"MAV CMD: {mav_cmd}")
    print(f"FILTER CMD: {filter_cmd}")
    print(f"SAFETY CHECKER CMD: {safety_checker_cmd}")

    # Launch SITL
    # add -L to a screen command to log to file
    os.system(f"screen -S {SCREEN_PREFIX}_{SCREEN_SITL_PREFIX} -dm {sitl_cmd}")
    # print(mav_cmd_with_filter)
    os.system(f"screen -S {SCREEN_PREFIX}_{SCREEN_MAV_PREFIX} -dm {mav_cmd}")
    os.system(
        f"screen -L -Logfile screenlog_filter -S {SCREEN_PREFIX}_{SCREEN_FILTER_PREFIX} -dm {filter_cmd}"
    )
    os.system(
        f"screen -L -Logfile screenlog_safetychecker -S {SCREEN_PREFIX}_{SCREEN_SAFETY_CHECKER_PREFIX} -dm bash -c '{safety_checker_cmd}'"
    )

    launched_services = [
        SCREEN_SITL_PREFIX,
        SCREEN_MAV_PREFIX,
        SCREEN_FILTER_PREFIX,
        SCREEN_SAFETY_CHECKER_PREFIX,
    ]

    # launch a vehicle script to if desired
    if not args.no_script:
        # we cd to run the vehicle script so we must modify the local script_params path
        script_params_file = os.path.join(os.getcwd(), args.script_params)
        script_cmd = f"cd {SPEED_TEST_DIR} && pwd && python -m aerpawlib --script {SPEED_TEST} --conn udp:127.0.0.1:{VEHICLE_SCRIPT_PORT} --vehicle {SCRIPT_VEHICLE_TYPE} --params {script_params_file} --skipoutput"
        print(f"SCRIPT CMD: {script_cmd}")

        os.system(
            f"screen -L -Logfile screenlog_vehiclescript -S {SCREEN_PREFIX}_{SCREEN_VEHICLE_SCRIPT_PREFIX} -dm bash -c '{script_cmd}'"
        )
        launched_services = launched_services.append(SCREEN_VEHICLE_SCRIPT_PREFIX)

    print("LAUNCHED SERVICES ON:")
    for service in launched_services:
        print(f"screen -R {SCREEN_PREFIX}_{service}")

    print("")

    if args.no_script:
        print(
            f"No vehicle script launched. Manually launch with address udp:127.0.0.1:{VEHICLE_SCRIPT_PORT}"
        )

    exit_commands = [
        f"screen -S {SCREEN_PREFIX}_{service} -X quit" for service in launched_services
    ]
    # print(exit_commands)

    print("press enter to quit:")
    input()
    for cmd in exit_commands:
        os.system(cmd)
