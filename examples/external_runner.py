"""
external_runner will run and interact with some processes

Usage:
    python -m aerpawlib --conn ... --vehicle generic --script external_runner
"""

import asyncio
import re

from aerpawlib.runner import BasicRunner, entrypoint
from aerpawlib.external import ExternalProcess

class MyScript(BasicRunner):
    @entrypoint
    async def do_stuff(self, vehicle):
        # spit out ls output
        ls = ExternalProcess("ls")
        await ls.start()
        while line := await ls.read_line():
            print(line)

        # spit out ps output and wait for some kind of python to show up (this script)
        ps = ExternalProcess("ps", params=["aux"])
        await ps.start()
        buff = await ps.wait_until_output(r'aerpaw')
        print(buff[-1])

        # talk interactively with cat
        cat = ExternalProcess("cat")
        await cat.start()
        await cat.send_input("dronegobrr\n")
        catized = await cat.read_line()
        print(catized)

        # ping loopback 5 times
        times = 5
        ping = ExternalProcess("ping", params=["127.0.0.1", "-c", str(times)])
        await ping.start()
        ping_re = re.compile(r".+icmp_seq=(?P<seq>\d+).+time=(?P<time>\d\.\d+) ms")
        while buff := await ping.wait_until_output(r'icmp_seq='):
            ping_re_match = ping_re.match(buff[-1])
            print(f"latency: {ping_re_match.group('time')}")
            if ping_re_match.group("seq") == str(times):
                break
