"""
Functionality exclusive to the AERPAW platform

An `AERPAW` object is automatically initialized and attached to a `Vehicle` upon
script initialization, if the runner automatically determines that it is being
run on the AERPAW platform.
"""

import asyncio
import base64
import requests

_DEFAULT_CVM_IP=""

OEO_MSG_SEV_INFO = "INFO"
OEO_MSG_SEV_WARN = "WARNING"
OEO_MSG_SEV_ERR = "ERROR"
OEO_MSG_SEV_CRIT = "CRITICAL"
_OEO_MSG_SEVS = [OEO_MSG_SEV_INFO, OEO_MSG_SEV_WARN, OEO_MSG_SEV_ERR, OEO_MSG_SEV_CRIT]

class AERPAW:
    _cvm_addr: str
    _connected: bool

    def __init__(self, cvm_addr=_DEFAULT_CVM_IP):
        self._cvm_addr = cvm_addr
        self._connected = self.attach_to_aerpaw_platform():

    def attach_to_aerpaw_platform(self) -> bool:
        """
        Attempts to attach this `AERPAW` object to the AERPAW platform/C-VM
        hosting this experiment. Returns bool depending on success.
        """
        try:
            requests.post(f"http://{cvm_addr}:12435/ping", timeout=1)
        except requests.exceptions.RequestException:
            return False
        return True

    def log_to_oeo(self, msg: str, severity: str=OEO_MSG_SEV_INFO):
        """
        Send `msg` to the OEO console, if connected. Prints message regardless
        of connection status.
        """
        print(msg)
        if not self._connected:
            return

        if severity not in _OEO_MSG_SEVS:
            raise Exception("severity provided for log_to_oeo not supported")
        encoded = base64.urlsafe_b64encode(msg.encode('utf-8'))
        try:
            requests.post(f"http://{cvm_addr}:12435/oeo_msg/{severity}/{encoded.decode('utf-8')}", timeout=3)
        except requests.exceptions.RequestException:
            print("unable to send previous message to OEO.")