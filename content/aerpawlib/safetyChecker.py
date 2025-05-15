import os
from typing import Dict, Tuple
import yaml
import zmq
import json
import zlib
import pickle
from argparse import ArgumentParser

from aerpawlib.util import doIntersect, inside, readGeofence, Coordinate

# Request key names supported by the safety checker server and client
SERVER_STATUS_REQ = "server_status_req"
VALIDATE_WAYPOINT_REQ = "validate_waypoint_req"
VALIDATE_CHANGE_SPEED_REQ = "validate_change_speed_req"
VALIDATE_TAKEOFF_REQ = "validate_takeoff_req"
VALIDATE_LANDING_REQ = "validate_landing_req"


# serialize a safety checker request
def serialize_request(request_function: str, params: list):
    raw_msg = json.dumps(
        {
            "request_function": request_function,
            "params": params,
        }
    )
    return serialize_msg(raw_msg)


# serialize a safety checker response
def serialize_response(request_function: str, result: bool, message: str = ""):
    raw_msg = json.dumps(
        {"request_function": request_function, "result": result, "message": message}
    )
    return serialize_msg(raw_msg)


def serialize_msg(raw_json):
    """Compress JSON message using zlib"""
    compressed_msg = zlib.compress(raw_json.encode("utf-8"))
    return compressed_msg


def deserialize_msg(compressed_msg):
    """Decompress JSON message using zlib"""
    raw_msg = zlib.decompress(compressed_msg).decode("utf-8")
    msg = json.loads(raw_msg)
    return msg


class SafetyCheckerClient:
    def __init__(self, addr: str, port: int):
        #  Prepare our context and sockets
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.socket.connect(f"tcp://{addr}:{port}")

    def sendRequest(self, msg):
        """Generic function to send a request to the safety checker server
        Sends the provided json message, then deserializes and decompresses
        the response from the server."""
        self.socket.send(msg)
        raw_msg = self.socket.recv()
        message = deserialize_msg(raw_msg)
        print(f"Received reply [{message}]")
        return message

    def parseResponse(self, response):
        """Parse response from SafetyCheckerServer such as the following sample
        {'request_function': 'validate_waypoint_req', 'result': True, 'message': ''}
        returns a tuple containing (result, message)"""
        return (response["result"], response["message"])

    def checkServerStatus(self):
        """Testing function to verify the safety checker server has launched
        with no issues"""
        msg = serialize_request(SERVER_STATUS_REQ, None)
        resp = self.sendRequest(msg)
        return self.parseResponse(resp)

    def validateWaypointCommand(self, curLoc: Coordinate, nextLoc: Coordinate):
        """
        Makes sure path from current location to next waypoint stays inside geofence and avoids no-go zones.
        Returns a tuple (bool, str)
        (False, <error message>) if the waypoint violates geofence or no-go zone constraints, else (True, "").
        """
        msg = serialize_request(
            VALIDATE_WAYPOINT_REQ, [curLoc.toJson(), nextLoc.toJson()]
        )
        resp = self.sendRequest(msg)
        return self.parseResponse(resp)

    def validateChangeSpeedCommand(self, newSpeed):
        """
        Makes sure the provided newSpeed lies within the configured vehicle constraints
        Returns (False, <error message>) if the speed violates constraints, else (True, "").
        """
        msg = serialize_request(VALIDATE_CHANGE_SPEED_REQ, [newSpeed])
        resp = self.sendRequest(msg)
        return self.parseResponse(resp)

    def validateTakeoffCommand(self, takeoffAlt, currentLat, currentLon):
        """
        Makes sure the takeoff altitude lies within the vehicle constraints
        Returns (False, <error message>) if the altitude violates constraints, else (True, "").
        """
        msg = serialize_request(
            VALIDATE_TAKEOFF_REQ, [takeoffAlt, currentLat, currentLon]
        )
        resp = self.sendRequest(msg)
        return self.parseResponse(resp)

    def validateLandingCommand(self, currentLat, currentLon):
        """
        Ensure the copter is attempting to land within 5 meters of the takeoff location
        Returns (False, <error message>) if the coper is not within 5 meters, else (True, "").
        """
        msg = serialize_request(VALIDATE_LANDING_REQ, [currentLat, currentLon])
        resp = self.sendRequest(msg)
        return self.parseResponse(resp)


class SafetyCheckerServer:

    # valid vehicle types
    VEHICLE_TYPES = ["rover", "copter"]
    # parameters required for all vehicle types
    REQUIRED_PARAMS = [
        "vehicle_type",
        "max_speed",
        "min_speed",
        "include_geofences",
        "exclude_geofences",
    ]
    # parameters required for copters
    REQUIRED_COPTER_PARAMS = ["max_alt", "min_alt"]

    def __init__(self, vehicle_config_filename: str, server_port=14580):
        # Construct map between function request strings and functions
        self.REQUEST_FUNCTIONS = {
            SERVER_STATUS_REQ: self.serverStatusHandler,
            VALIDATE_WAYPOINT_REQ: self.validateWaypointHandler,
            VALIDATE_CHANGE_SPEED_REQ: self.validateChangeSpeedHandler,
            VALIDATE_TAKEOFF_REQ: self.validateTakeoffHandler,
            VALIDATE_LANDING_REQ: self.validateLandingHandler,
        }

        vehicle_config_file = open(vehicle_config_filename, "r")
        config = yaml.safe_load(vehicle_config_file)

        self.validate_config(config, vehicle_config_filename)

        self.vehicle_type = config["vehicle_type"]

        (vehicle_config_dir, _) = os.path.split(vehicle_config_filename)
        self.include_geofences = [
            readGeofence(os.path.join(vehicle_config_dir, geofence))
            for geofence in config["include_geofences"]
        ]
        # No go zones to exclude from the geofenced area
        self.exclude_geofences = [
            readGeofence(os.path.join(vehicle_config_dir, geofence))
            for geofence in config["exclude_geofences"]
        ]
        self.max_speed = config["max_speed"]
        self.min_speed = config["min_speed"]

        # Only max altitude for copters
        if self.vehicle_type == "copter":
            self.max_alt = config["max_alt"]
            self.min_alt = config["min_alt"]

        # start server BLOCKING
        self.start_server(server_port)

    def start_server(self, port):
        context = zmq.Context()
        socket = context.socket(zmq.REP)
        socket.bind(f"tcp://*:{port}")

        print("waiting for messages")
        while True:
            raw_msg = socket.recv()
            message = deserialize_msg(raw_msg)
            print(f"Received request: {message}")
            try:
                function_name = message["request_function"]
                req_function = self.REQUEST_FUNCTIONS[function_name]
                params = message["params"]
                if params is None:
                    response = req_function(None)
                else:
                    response = req_function(*params)
                socket.send(response)
            except KeyError as e:
                socket.send(f"Unimplemented function request <{function_name}>")
            except Exception as e:
                socket.send(b"Unknown error!")
                raise (e)

    def validate_config(self, config: Dict, vehicle_config_filename: str):
        """Ensures that the provided config dict contains all necessary parameters.
        Raises an exception if the config is invalid"""
        # Check if all required params exist
        for param in self.REQUIRED_PARAMS:
            if param not in config:
                raise Exception(
                    f"Required parameter {param} not found in {vehicle_config_filename}!"
                )

        # Ensure the vehicle type is valid
        if config["vehicle_type"] not in self.VEHICLE_TYPES:
            raise Exception(
                f"Vehicle type in {vehicle_config_filename} is invalid! Must be one of {self.VEHICLE_TYPES}"
            )

        # If the vehicle is a copter ensure copter-specific required params exist
        if config["vehicle_type"] == "copter":
            for param in self.REQUIRED_COPTER_PARAMS:
                if param not in config:
                    raise Exception(
                        f"Required copter parameter {param} not found in {vehicle_config_filename}!"
                    )

    def validateWaypointCommand(
        self, curLoc: Coordinate, nextLoc: Coordinate
    ) -> Tuple[bool, str]:
        """
        Makes sure path from current location to next waypoint stays inside geofence and avoids no-go zones.
        Returns a tuple (bool, str)
        (False, <error message>) if the waypoint violates geofence or no-go zone constraints, else (True, "").
        """
        print(f"Validating {nextLoc}")

        # Makes sure altitude of next waypoint is within regulations
        if self.vehicle_type == "copter":
            if nextLoc.alt < self.min_alt or nextLoc.alt > self.max_alt:
                return (
                    False,
                    "Invalid waypoint. Altitude of %s m is not within restrictions! ABORTING!"
                    % nextLoc.alt,
                )

        # Makes sure next waypoint is inside one of the include geofences
        inside_geofence = False
        for geofence in self.include_geofences:
            if inside(nextLoc.lon, nextLoc.lat, geofence):
                inside_geofence = True
                break
        if inside_geofence == False:
            return (
                False,
                "Invalid waypoint. Waypoint (%s,%s) is outside of the geofence. ABORTING!"
                % (nextLoc.lat, nextLoc.lon),
            )
        # Makes sure next waypoint is not in a no-go zone
        for zone in self.exclude_geofences:
            if inside(nextLoc.lon, nextLoc.lat, zone):
                return (
                    False,
                    "Invalid waypoint. Waypoint (%s,%s) is inside a no-go zone. ABORTING!"
                    % (nextLoc.lat, nextLoc.lon),
                )
        # Makes sure path between two points does not leave geofence
        for i in range(len(geofence) - 1):
            if doIntersect(
                geofence[i]["lon"],
                geofence[i]["lat"],
                geofence[i + 1]["lon"],
                geofence[i + 1]["lat"],
                curLoc.lon,
                curLoc.lat,
                nextLoc.lon,
                nextLoc.lat,
            ):
                return (
                    False,
                    "Invalid waypoint. Path from (%s,%s) to waypoint (%s,%s) leaves geofence. ABORTING!"
                    % (curLoc.lat, curLoc.lon, nextLoc.lat, nextLoc.lon),
                )

        # Makes sure path between two points does not enter no-go zone
        for zone in self.exclude_geofences:
            for i in range(len(zone) - 1):
                if doIntersect(
                    zone[i]["lon"],
                    zone[i]["lat"],
                    zone[i + 1]["lon"],
                    zone[i + 1]["lat"],
                    curLoc.lon,
                    curLoc.lat,
                    nextLoc.lon,
                    nextLoc.lat,
                ):
                    return (
                        False,
                        "Invalid waypoint. Path from (%s,%s) to waypoint (%s,%s) enters no-go zone. ABORTING!"
                        % (curLoc.lat, curLoc.lon, nextLoc.lat, nextLoc.lon),
                    )

        # Next waypoint location is valid
        return (True, "")

    def validateChangeSpeedCommand(self, newSpeed) -> Tuple[bool, str]:
        """
        Makes sure the provided newSpeed lies within the configured vehicle constraints
        Returns (False, <error message>) if the speed violates constraints, else (True, "").
        """
        if newSpeed > self.max_speed:
            return (
                False,
                "Invalid speed (%s) greater than maximum (%s)"
                % (newSpeed, self.max_speed),
            )
        if newSpeed < self.min_speed:
            return (
                False,
                "Invalid speed (%s) less than minimum (%s)"
                % (newSpeed, self.min_speed),
            )
        # New speed is valid
        return (True, "")

    def validateTakeoffCommand(self, takeoffAlt, currentLat, currentLon):
        """
        Makes sure the takeoff altitude lies within the vehicle constraints
        Returns (False, <error message>) if the altitude violates constraints, else (True, "").
        """
        # Makes sure altitude is within regulations
        if self.vehicle_type == "copter":
            if takeoffAlt < self.min_alt or takeoffAlt > self.max_alt:
                return (False, "Invalid takeoff altitude of %s m." % takeoffAlt)
        # Save takeoff location for validating landing
        self.takeoff_location = Coordinate(currentLat, currentLon, alt=0)
        # Takeoff command is valid
        return (True, "")

    def validateLandingCommand(self, currentLat, currentLon):
        """
        Ensure the copter is attempting to land within 5 meters of the takeoff location
        Returns (False, <error message>) if the coper is not within 5 meters, else (True, "").
        """
        # # approximate radius of earth in km
        # R = 6373.0
        # # Calculate distance between takeoff and landing
        # lat1 = radians(currentLat)
        # lon1 = radians(currentLon)
        # lat2 = radians(self.takeoff_location[0])
        # lon2 = radians(self.takeoff_location[1])
        # dlon = lon2 - lon1
        # dlat = lat2 - lat1
        # a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
        # c = 2 * atan2(sqrt(a), sqrt(1 - a))
        # distance = R * c * 1000
        currentLocation = Coordinate(currentLat, currentLon, alt=0)
        distance = self.takeoff_location.ground_distance(currentLocation)

        if distance > 5:
            return (
                False,
                "Invalid landing location. Must be within 5 meters of takeoff location. Attempted landing location (%s,%s) is %f meters from takeoff location."
                % (currentLat, currentLon, distance),
            )
        # Landing command is valid
        return (True, "")

    #############################
    ## Client Request Handlers ##
    #############################
    def serverStatusHandler(self, *_params):
        msg = serialize_response(request_function=SERVER_STATUS_REQ, result=True)
        return msg

    def validateWaypointHandler(self, curLocJSON, nextLocJSON, *_params):
        curLocDict = json.loads(curLocJSON)
        nextLocDict = json.loads(nextLocJSON)
        curLoc = Coordinate(
            lat=curLocDict["lat"], lon=curLocDict["lon"], alt=curLocDict["alt"]
        )
        nextLoc = Coordinate(
            lat=nextLocDict["lat"], lon=nextLocDict["lon"], alt=nextLocDict["alt"]
        )

        result, message = self.validateWaypointCommand(curLoc, nextLoc)
        msg = serialize_response(
            request_function=VALIDATE_WAYPOINT_REQ, result=result, message=message
        )
        return msg

    def validateChangeSpeedHandler(self, newSpeed, *_params):
        result, message = self.validateChangeSpeedCommand(newSpeed)
        msg = serialize_response(
            request_function=VALIDATE_CHANGE_SPEED_REQ, result=result, message=message
        )
        return msg

    def validateTakeoffHandler(self, takeoffAlt, currentLat, currentLon, *_params):
        result, message = self.validateTakeoffCommand(
            takeoffAlt, currentLat, currentLon
        )
        msg = serialize_response(
            request_function=VALIDATE_TAKEOFF_REQ, result=result, message=message
        )
        return msg

    def validateLandingHandler(self, currentLat, currentLon, *_params):
        result, message = self.validateLandingCommand(currentLat, currentLon)
        msg = serialize_response(
            request_function=VALIDATE_LANDING_REQ, result=result, message=message
        )
        return msg


if __name__ == "__main__":
    parser = ArgumentParser(
        description="safetyChecker - Launch a safety checker server"
    )
    parser.add_argument(
        "--port", help="Port for communication between client and server", required=True
    )
    parser.add_argument(
        "--vehicle_config",
        help="Path to YAML file containing geofences and vehicle constraints",
        required=True,
    )
    args, _ = parser.parse_known_args()

    # This call blocks
    server = SafetyCheckerServer(args.vehicle_config, server_port=args.port)
