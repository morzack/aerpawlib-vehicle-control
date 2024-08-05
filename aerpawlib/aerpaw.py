"""
Functionality exclusive to the AERPAW platform

The AERPAW_Platform singleton will automatically detect if a script is being
run in an AERPAW experiment, in which case it enables additional AERPAW
functionality.
"""

import asyncio
import base64
import requests

_DEFAULT_CVM_IP = "192.168.32.25"
_DEFAULT_CVM_PORT = 12435

OEO_MSG_SEV_INFO = "INFO"
OEO_MSG_SEV_WARN = "WARNING"
OEO_MSG_SEV_ERR = "ERROR"
OEO_MSG_SEV_CRIT = "CRITICAL"
_OEO_MSG_SEVS = [OEO_MSG_SEV_INFO, OEO_MSG_SEV_WARN, OEO_MSG_SEV_ERR, OEO_MSG_SEV_CRIT]

class AERPAW:
    _cvm_addr: str
    _connected: bool

    _connection_warning_displayed = False

    def __init__(self, cvm_addr=_DEFAULT_CVM_IP, cvm_port=_DEFAULT_CVM_PORT):
        self._cvm_addr = cvm_addr
        self._cvm_port = cvm_port
        self._connected = self.attach_to_aerpaw_platform()

    def attach_to_aerpaw_platform(self) -> bool:
        """
        Attempts to attach this `AERPAW` object to the AERPAW platform/C-VM
        hosting this experiment. Returns bool depending on success.
        """
        try:
            requests.post(f"http://{self._cvm_addr}:{self._cvm_port}/ping", timeout=1)
        except requests.exceptions.RequestException:
            return False
        return True

    def _display_connection_warning(self):
        if self._connection_warning_displayed:
            return
        print("[aerpawlib] INFO: the user script has attempted to use AERPAW platform functionality without being in the AERPAW environment")
        self._connection_warning_displayed = True

    def log_to_oeo(self, msg: str, severity: str=OEO_MSG_SEV_INFO):
        """
        Send `msg` to the OEO console, if connected. Prints message regardless
        of connection status.
        """
        print(msg)
        if not self._connected:
            self._display_connection_warning()
            return

        if severity not in _OEO_MSG_SEVS:
            raise Exception("severity provided for log_to_oeo not supported")
        encoded = base64.urlsafe_b64encode(msg.encode('utf-8'))
        try:
            requests.post(f"http://{self._cvm_addr}:{self._cvm_port}/oeo_msg/{severity}/{encoded.decode('utf-8')}", timeout=3)
        except requests.exceptions.RequestException:
            print("unable to send previous message to OEO.")

    def _checkpoint_build_request(self, var_type, var_name):
        return f"http://{self._cvm_addr}:{self._cvm_port}/checkpoint/{var_type}/{var_name}"

    # NOTE: unlike the above functionality, all checkpoint functions will cause an
    # exception if they are run while not in the AERPAW platform, as there isn't a way
    # to "recover" while maintaining the function's API contract
    
    def checkpoint_reset_server(self):
        """
        Reset the AERPAW checkpoint server.

        This function should be called at the start of an experiment by an E-VM script
        to ensure that no stored state remains between experiment runs.
        """
        if not self._connected:
            self._display_connection_warning()
            raise Exception("AERPAW checkpoint functionality only works in AERPAW environment")
        response = requests.post(f"http://{self._cvm_addr}:{self._cvm_port}/checkpoint/reset")
        if response.status_code != 200:
            raise Exception("error when resetting checkpoint server")
    
    def checkpoint_set(self, checkpoint_name: str):
        """
        Set a checkpoint in the AERPAW checkpoint system with name `checkpoint_name`
        """
        if not self._connected:
            self._display_connection_warning()
            raise Exception("AERPAW checkpoint functionality only works in AERPAW environment")
        response = requests.post(self._checkpoint_build_request("bool", checkpoint_name))
        if response.status_code != 200:
            raise Exception("error when posting to checkpoint server")

    def checkpoint_check(self, checkpoint_name: str) -> bool:
        """
        See if a checkpoint has been set in the AERPAW checkpoint system with name
        `checkpoint_name`.

        Returns `True` if it has been set, `False` otherwise
        """
        if not self._connected:
            self._display_connection_warning()
            raise Exception("AERPAW checkpoint functionality only works in AERPAW environment")
        response = requests.get(self._checkpoint_build_request("bool", checkpoint_name))
        if response.status_code != 200:
            raise Exception("error when getting from checkpoint server")
        response_content = response.content.decode()
        if response_content == "True":
            return True
        elif response_content == "False":
            return False
        raise Exception(f"malformed content in response from server: {response_content}")

    def checkpoint_increment_counter(self, counter_name: str):
        """
        Increment a counter in the AERPAW checkpoint system with name `counter_name`.
        """
        if not self._connected:
            self._display_connection_warning()
            raise Exception("AERPAW checkpoint functionality only works in AERPAW environment")
        response = requests.post(self._checkpoint_build_request("int", counter_name))
        if response.status_code != 200:
            raise Exception("error when posting to checkpoint server")

    def checkpoint_check_counter(self, counter_name: str) -> int:
        """
        Get the current state of a counter in the AERPAW checkpoint system with name
        `counter_name`

        An un-incremented counter will always start at 0, regardless of if it has been
        interacted with.
        """
        if not self._connected:
            self._display_connection_warning()
            raise Exception("AERPAW checkpoint functionality only works in AERPAW environment")
        response = requests.get(self._checkpoint_build_request("int", counter_name))
        if response.status_code != 200:
            raise Exception("error when getting from checkpoint server")
        response_content = response.content.decode()
        try:
            return int(response_content)
        except TypeError:
            raise Exception(f"malformed content in response from server: {response_content}")

    def checkpoint_set_string(self, string_name: str, value: str):
        """
        Set the value of a string in the AERPAW checkpoint system's key:value store with
        name `string_name` and value `value`
        """
        if not self._connected:
            self._display_connection_warning()
            raise Exception("AERPAW checkpoint functionality only works in AERPAW environment")
        response = requests.post(self._checkpoint_build_request("string", string_name) + f"?val={value}")
        if response.status_code != 200:
            raise Exception("error when posting to checkpoint server")

    def checkpoint_check_string(self, string_name: str) -> str:
        """
        Get the value of a key:value pair in AERPAW's key:value store with key
        `string_name`
        """
        if not self._connected:
            self._display_connection_warning()
            raise Exception("AERPAW checkpoint functionality only works in AERPAW environment")
        response = requests.get(self._checkpoint_build_request("string", string_name))
        if response.status_code != 200:
            raise Exception("error when getting from checkpoint server")
        response_content = response.content.decode()
        return response_content

AERPAW_Platform = AERPAW()