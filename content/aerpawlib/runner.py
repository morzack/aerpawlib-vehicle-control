"""
Collection of execution frameworks that can be extended to make scripts
runnable using aerpawlib. The most basic framework is `Runner` -- any custom
frameworks *must* extend it to be executable.
"""

import asyncio
from enum import Enum, auto
import inspect
from typing import Callable, Dict, List

from .vehicle import Vehicle

class Runner:
    """
    Base execution framework -- generally, this should only be used to build
    new runners. Only exposes the base functions needed to interface with
    aerpawlib's runner.
    """
    async def run(self, _: Vehicle):
        """
        Run the script that's been loaded in -- this is what implements the
        core logic of whatever's being run. The `aerpawlib.vehicle.Vehicle`
        passed in will have been initialized externally by the launch script.

        For other runners provided with aerpawlib (ex: `StateMachine`,
        `BasicRunner`), you should avoid overriding this.
        """
        pass
    
    def initialize_args(self, _: List[str]):
        """
        Can be overridden to parse and handle extra command line arguments
        passed to the runner. The easiest way to interact with them is through
        `argparse` -- check out `examples/squareoff_logging.py` for an example
        of this.

        This is expected to be overridden by the user script if it is used.
        """
        pass
    
    def cleanup(self):
        """
        Any additional user-provided functionality that is called when the
        script exits should be implemented here.

        This is expected to be overridden by the user script if it is used.
        """
        pass

_Runnable = Callable[[Runner, Vehicle], str]

def entrypoint(func):
    """
    Decorator used to identify the entry point used by `BasicRunner` driven
    scripts.

    The function decorated by this is expected to be `async`
    """
    func._entrypoint = True
    return func

class BasicRunner(Runner):
    """
    BasicRunners have a single entry point (specified by `entrypoint`) that is
    executed when the script is run. The function provided can be anything, as
    it will be run in parallel to background services used by aerpawlib.

    For an example of a minimum viable `BasicRunner`, check out
    `examples/basic_runner.py`
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
    
    async def run(self, runner: Runner, vehicle: Vehicle) -> str:
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
    Decorator used to specify a state used by a `StateMachine`. Functions
    decorated using this are expected to be given a name and return a string
    that is the name of the next state to transition to/run. If no next state
    is returned, then the `StateMachine` will (gracefully) stop.

    The function decorated by this is expected to be `async`
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
    Timed state. Will wait `duration` seconds before transitioning to the next
    state. If `loop` is true, the decorated function will be continuously
    called until the duration has expired.

    This guarentees that the state will persist for *at least* `duration`
    seconds, so timed states will continue running for longer than that if the
    decorated function lasts longer than the timer.

    The function decorated by this is expected to be `async`
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

_BackgroundTask = Callable[[Runner, Vehicle], None]

def background(func):
    """
    Designate a function to be run in parallel to a given StateMachine.
    By `asyncio`'s design, background function's don't have to take
    thread-safety into consideration.

    The function decorated by this is expected to be `async`

    When using @background, make sure to incorporate some kind of delay.
    Without any delay, an @background thread can eat all the python
    instance's CPU
    """
    func._is_background = True
    return func

_InitializationTask = Callable[[Runner, Vehicle], None]

def at_init(func):
    """
    Designate a function to be run at vehicle initialization and before vehicle
    arming (or at least the prompt to arm). The vehicle will not be armable
    until after all `at_init` functions finish executing.
    
    The function designated should have a signature that accepts a Vehicle.

    The function decorated by this is expected to be `async`. If there are
    multiple functions marked with `at_init`, they will all be run at the same
    time.
    """
    func._run_at_init = True
    return func

_STATE_DELAY = 0.01 # s between each time the state update is called

class StateMachine(Runner):
    """
    A `StateMachine` is a type of runner that consists of various states
    declared using the `state` decorator. Each state, when run, should return a
    string that is the name of the next state to execute. If no next state is
    provided, it's assumed that the state machine is done running.

    `StateMachine` also supports `background` tasks, which are run in parallel
    to any state execution.

    For an example of a `StateMachine` in action, check out
    `examples/squareoff_logging.py`
    """

    _states: Dict[str, _State]
    _background_tasks: List[_BackgroundTask]
    _initialization_tasks: List[_InitializationTask]
    _entrypoint: str
    _current_state: str
    _running: bool

    def _build(self):
        self._states = {}
        self._background_tasks = []
        self._initialization_tasks = []
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
            if hasattr(method, "_run_at_init"):
                self._initialization_tasks.append(method)
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
        

        if len(self._initialization_tasks) != 0:
            await asyncio.wait({f(vehicle) for f in self._initialization_tasks})
        
        print("[aerpawlib] Initialization tasks complete. Waiting for safety pilot to arm")
        await vehicle._initialize_postarm()
        
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
        """
        Call `stop` to stop the execution of the `StateMachine` after
        completion of the current state. This is equivalent to returning `None`
        at the end of a state's execution.
        """
        self._running = False

# helper functions for working with asyncio code

# this is cheap but lets us write code in a more "researcher friendly" way
in_background = asyncio.ensure_future
sleep = asyncio.sleep
