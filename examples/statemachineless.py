from aerpaw.staterunner import command, StateMachine
from aerpaw.vehicle import Vehicle

class MyScript(StateMachine):
    # let's say they don't like our state machine
    @command(name="no state machine plz", first=True)
    def run_my_stuff(self, vehicle: Vehicle):
        # they just put everything into one command
        # we could even write a wrapper for this
        vehicle.do_something("i'm doing a thing")

        # maybe it moves somewhere now
        stateful_value = vehicle.get_something(1)

        vehicle.do_something(f"i just saw {stateful_value}, stopping")
        
        # self.stop() # <- this is implicitly called if the function returns None