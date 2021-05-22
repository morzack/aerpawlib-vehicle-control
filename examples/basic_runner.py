from aerpawlib.runner import BasicRunner, entrypoint
from aerpawlib.vehicle import Vehicle

class MyScript(BasicRunner):
    @entrypoint
    def do_stuff(self, vehicle: Vehicle):
        print("woo, i'm running now")
