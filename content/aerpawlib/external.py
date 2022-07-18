"""
Utilities allowing for aerpawlib scripts to interact with external processes
running in a userspace.
"""

import asyncio
from typing import List
import re

class ExternalProcess:
    """
    Object allowing for interaction with a process spawned by this script and
    run asynchronously to the aerpawlib script. Allows for basic interaction
    with stdio and stdout as well as dynamic passing of arguments.
    """
    def __init__(self, executable: str, params=[], stdin: str=None, stdout: str=None):
        """
        Prepare external process for execution. Does NOT execute process, you
        must call `start()`.

        `params` should be the parameters that will be passed to the process.
        Split the list where there would be spaces in the command line version.
        """
        self._executable = executable
        self._params = params
        self._stdin = stdin
        self._stdout = stdout
        
    async def start(self):
        """
        Start the executable in an asyncronous process
        """
        executable = self._executable
        executable += " " + " ".join(self._params)
        if not self._stdin is None:
            executable += f" < {self._stdin}"
        if not self._stdout is None:
            executable += f" > {self._stdout}"
            
        self.process = await asyncio.create_subprocess_shell(
                executable,
                stdout=None if self._stdout is not None else asyncio.subprocess.PIPE,
                stdin=None if self._stdin is not None else asyncio.subprocess.PIPE)
    
    async def read_line(self) -> str:
        """
        Read one line from the stdout buffer. Returns None if process has stopped.
        """
        if not self.process.stdout:
            return None
        out = await self.process.stdout.readline()
        return out.decode('ascii').rstrip()

    async def send_input(self, data: str):
        """
        Send a string to the process's stdin
        """
        self.process.stdin.write(data.encode())
        await self.process.stdin.drain()

    async def wait_until_terminated(self):
        """
        Idles until process is complete
        """
        await self.process.wait()

    async def wait_until_output(self, output_regex) -> List[str]:
        """
        block and wait until we see the output_regex regular expression show up
        in a line of the output stream (only works w/out stdout set)
        
        Returns all lines consumes up until and including regex
        
        Use an r"string" as output_regex
        
        Will exit only if process terminates or pattern is matched
        """
        buff = []
        while True:
            out = await self.read_line()
            buff.append(out)
            if re.search(output_regex, out):
                return buff
