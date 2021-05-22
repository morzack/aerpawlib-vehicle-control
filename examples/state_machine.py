from aerpawlib.runner import StateMachine, state
from aerpawlib.vehicle import Vehicle

class MyScript(StateMachine):
    @state(name="start", first=True)
    def start(self, vehicle: Vehicle):
        print("start")
        return "loop"

    @state(name="loop")
    def loop(self, vehicle: Vehicle):
        print("looping")
        return "end"

    @state(name="end")
    def end(self, vehicle: Vehicle):
        print("done")
