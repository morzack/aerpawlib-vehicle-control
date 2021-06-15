import asyncio
from enum import Enum, auto
import inspect
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

class _StateType(Enum):
    STANDARD = auto()
    TIMED = auto()

class _State:
    _name: str
    _func: _Runnable

    def __init__(self, func: _Runnable, name: str):
        self._name = name
        self._func = func
    
    async def run(self, runner: _Runner, vehicle: Vehicle) -> str:
        if self._func._state_type == _StateType.STANDARD:
            return await self._func.__func__(runner, vehicle)
        elif self._func._state_type == _StateType.TIMED:
            running = True
            async def _bg():
                nonlocal running
                last_state = ""
                while running:
                    last_state = await self._func.__func__(runner, vehicle)
                    if not self._func._state_loop:
                        running = False
                    await asyncio.sleep(_STATE_DELAY)
                return last_state
            r = asyncio.ensure_future(_bg())
            # order here is important and stops a race condition
            await asyncio.sleep(self._func._state_duration)
            running = False
            next_state = await r 
            return next_state
        return ""

def state(name: str, first: bool=False):
    """
    Standard state. Will finish executing when the func provided in is done, at
    which point it transitions to the state (string) returned.
    """
    if name == "":
        raise Exception("state name can't be \"\"")
    def decorator(func):
        func._is_state = True
        func._state_name = name
        func._state_first = first
        func._state_type = _StateType.STANDARD
        return func
    return decorator

def timed_state(name: str, duration: float, loop=False, first: bool=False):
    """
    Timed state. Will wait duration seconds before transitioning to the next
    state. If loop is true, the function will be repeatedly called until the
    timer is complete. This guarentees that the state will persist for at least
    *duration* seconds.
    """
    def decorator(func):
        func._is_state = True
        func._state_name = name
        func._state_first = first
        func._state_type = _StateType.TIMED
        func._state_duration = duration
        func._state_loop = loop
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
                print(self._current_state)
                raise Exception("Illegal state")
            self._current_state = await self._states[self._current_state].run(self, vehicle)
            if self._current_state is None:
                self.stop()
            await asyncio.sleep(_STATE_DELAY)
        self.cleanup()

    def stop(self):
        self._running = False

# helper functions for working with asyncio code

# this is cheap but lets us write code in a more "researcher friendly" way
in_background = asyncio.ensure_future
sleep = asyncio.sleep
