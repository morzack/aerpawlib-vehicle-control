# fake vehicle. we'd selectively expose dronekit with this
class Vehicle:
    def do_something(self, thing):
        print(thing)
    
    def get_something(self, thing):
        return thing