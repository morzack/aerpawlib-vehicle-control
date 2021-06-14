import asyncio
import inspect
import time
from typing import Callable, Dict, List

from .vehicle import Vehicle

class _Runner:
    async def run(self, _: Vehicle):
        """
        Run the script that's been loaded in -- impl dependent
        """
        pass
    
    def initialize_args(self, _: List[str]):
        """
        Can be overridden to parse and handle extra args passed to the runner
        """
        pass
    
    def cleanup(self):
        """
        Additional user-provided functionality that is called when the script
        exits
        """
        pass

_Runnable = Callable[[_Runner, Vehicle], str]

def entrypoint(func):
    func._entrypoint = True
    return func

class BasicRunner(_Runner):
    """
    BasicRunners have a single entry point (specified by @entrypoint) that is
    executed when the script is run. The function provided can be anything,
    blocking doesn't matter.
    """
    def _build(self):
        for _, method in inspect.getmembers(self):
            if not inspect.ismethod(method):
                continue
            if hasattr(method, "_entrypoint"):
                self._entry = method

    async def run(self, vehicle: Vehicle):
        self._build()
        if hasattr(self, "_entry"):
            await self._entry.__func__(self, vehicle)
        else:
            raise Exception("No @entrypoint declared")

class _State:
    _name: str
    _func: _Runnable

    def __init__(self, func: _Runnable, name: str):
        self._name = name
        self._func = func

    def __call__(self, runner: _Runner, vehicle: Vehicle):
        return self._func.__func__(runner, vehicle)

def state(name: str, first: bool=False):
    def decorator(func):
        func._is_state = True
        func._state_name = name
        func._state_first = first
        return func
    return decorator

_BackgroundTask = Callable[[_Runner, Vehicle], None]

def background(func):
    """
    Designate a function to be run in parallel to a given StateMachine. This
    functionality only works if the state machine is designed to be non-blocking

    so, there's an implcit TODO here to use threading.
    """
    func._is_background = True
    return func

_STATE_DELAY = 0.01 # s between each time the state update is called

class StateMachine(_Runner):
    """
    A StateMachine is a type of runner that consists of various states declared
    using @state. Each state when run should return a string that is the name
    of the next state to execute. If no next state is provided, it's assumed
    that the state machine is done running.

    A note about blocking: no state in the state machine should be blocking
    (i.e. don't include while loops or time.sleeps, unless you know what you're
    doing). By doing that, you're blocking the entire runner thread, which in
    turn blocks other things such as background tasks. This will be addressed
    later when proper threading/async patterns are implemented.
    """

    _states: Dict[str, _State]
    _background_tasks: List[_BackgroundTask]
    _entrypoint: str
    _current_state: str
    _running: bool

    def _build(self):
        self._states = {}
        self._background_tasks = []
        for _, method in inspect.getmembers(self):
            if not inspect.ismethod(method):
                continue
            if hasattr(method, "_is_state"):
                self._states[method._state_name] = _State(method, method._state_name)
                if method._state_first and not hasattr(self, "_entrypoint"):
                    self._entrypoint = method._state_name
                elif method._state_first and hasattr(self, "_entrypoint"):
                    raise Exception("There may only be one initial state")
            if hasattr(method, "_is_background"):
                self._background_tasks.append(method)
        if not self._entrypoint:
            raise Exception("There is no initial state")
    
    async def _start_background_tasks(self, vehicle: Vehicle):
        for task in self._background_tasks:
            async def _task_runner(t=task):
                while self._running:
                    await t.__func__(self, vehicle)
            asyncio.ensure_future(_task_runner())

    async def run(self, vehicle: Vehicle):
        self._build()
        assert self._entrypoint
        self._current_state = self._entrypoint
        self._running = True
        await self._start_background_tasks(vehicle)
        while self._running:
            if self._current_state not in self._states:
                raise Exception("Illegal state")
            self._current_state = await self._states[self._current_state](self, vehicle)
            if self._current_state is None:
                self.stop()
            await asyncio.sleep(_STATE_DELAY)
        self.cleanup()

    def stop(self):
        self._running = False
