from aerpaw.staterunner import command, StateMachine
from aerpaw.vehicle import Vehicle

class MyScript(StateMachine):
    state = 0

    # example commands -- the first command has a special flag
    # also note that they return the next state
    @command(name="thing_1", first=True)
    def do_thing_1(self, vehicle: Vehicle):
        vehicle.do_something("thing 1!")
        return "thing_2"

    @command(name="thing_2")
    def do_another_thing(self, vehicle: Vehicle):
        vehicle.do_something("I'm doing another thing!")
        return "loop time"

    @command(name="loop time")
    def loop_with_mem(self, vehicle: Vehicle):
        # so that you can have non-stateless commands
        self.state += 1
        vehicle.do_something(f"I've done the loop {self.state} times!")
        return "done" if self.state == 5 else "loop time"

    @command(name="done")
    def finish(self, vehicle: Vehicle):
        vehicle.do_something("I'm done now!")
        self.stop()