from typing import Callable, Dict

from aerpaw.vehicle import Vehicle

# states are simply functions that are presented with a vehicle and return a str for the next state
# note that class level variables can be used to hold state
_State = Callable[[Vehicle], str]
_Routine = Dict[str, _State]

class OtherThing:
    pass

class StateMachine:
    _states: _Routine={}
    _initial_state: str
    _current_state: str
    _running: bool

    # basic state machine runner (just call func that matches state in dict)
    def run(self, vehicle: Vehicle):
        assert self._initial_state
        self._current_state = self._initial_state
        self._running = True
        while self._running:
            assert self._current_state in self._states
            self._current_state = self._states[self._current_state](self, vehicle)
            if self._current_state == None:
                self.stop()

    def stop(self):
        self._running = False

# we use this to register commands that the experimenters use
# stored as name: command [func] data struct for state machine
def command(name: str, first: bool=False):
    def decorator(func):
        def wrapper(*args, **kwargs):
            return func(*args, **kwargs)
        StateMachine._states[name] = wrapper
        if first:
            StateMachine._initial_state = name
        return wrapper
    return decorator
