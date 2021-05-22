import inspect
from typing import Callable, Dict

from .vehicle import Vehicle

class _Runner:
    def run(self, _: Vehicle):
        """
        Run the script that's been loaded in -- impl dependent
        """
        pass

_Runnable = Callable[[_Runner, Vehicle], str]

def entrypoint(func):
    func._entrypoint = True
    return func

class BasicRunner(_Runner):
    def _build(self):
        for _, method in inspect.getmembers(self):
            if not inspect.ismethod(method):
                continue
            if hasattr(method, "_entrypoint"):
                self._entry = method

    def run(self, vehicle: Vehicle):
        self._build()
        if hasattr(self, "_entry"):
            self._entry.__func__(self, vehicle)
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

class StateMachine(_Runner):
    _states: Dict[str, _State]
    _entrypoint: str
    _current_state: str
    _running: bool

    def _build(self):
        self._states = {}
        for _, method in inspect.getmembers(self):
            if not inspect.ismethod(method):
                continue
            if hasattr(method, "_is_state"):
                self._states[method._state_name] = _State(method, method._state_name)
                if method._state_first and not hasattr(self, "_entrypoint"):
                    self._entrypoint = method._state_name
                elif method._state_first and hasattr(self, "_entrypoint"):
                    raise Exception("There may only be one initial state")
        if not self._entrypoint:
            raise Exception("There is no initial state")

    def run(self, vehicle: Vehicle):
        self._build()
        assert self._entrypoint
        self._current_state = self._entrypoint
        self._running = True
        while self._running:
            if self._current_state not in self._states:
                raise Exception("Illegal state")
            self._current_state = self._states[self._current_state](self, vehicle)
            if self._current_state is None:
                self.stop()

    def stop(self):
        self._running = False
