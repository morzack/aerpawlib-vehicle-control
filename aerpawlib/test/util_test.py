from aerpawlib.util import Coordinate, SafetyChecker, VectorNED


def test_coordinate():
    a = Coordinate(35.771634, -78.674109)
    b = Coordinate(35.770772, -78.674825, 100)

    assert round(a.ground_distance(b)) == 116

    assert round(a.distance(b)) == 153

    assert round(a.bearing(b)) == 220

    assert round(a.bearing(a + VectorNED(100, 0, 0))) == 0
    assert round(a.bearing(a + VectorNED(-100, 0, 0))) == 180
    assert round(a.bearing(a + VectorNED(0, 100, 0))) == 90
    assert round(a.bearing(a + VectorNED(0, -100, 0))) == 270

    assert round(a.distance(a + VectorNED(100, 0, 0))) == 100
    assert round(a.distance(a + VectorNED(100, 100, 0))) == 141
    
    assert round(a.distance(a - VectorNED(100, 0, 0))) == 100
    assert round(a.distance(a - VectorNED(100, 100, 0))) == 141

    delta = a - (a - VectorNED(100, 50, -25))
    assert isinstance(delta, VectorNED)
    assert round(delta.north) == 100
    assert round(delta.east) == 50
    assert round(delta.down) == -25


def test_vectorned():
    a = VectorNED(1, 0, 0)
    b = VectorNED(1, -1, 1)

    delta = a + b
    assert delta.north == 2
    assert delta.east == -1
    assert delta.down == 1
    
    delta = a - b
    assert delta.north == 0
    assert delta.east == 1
    assert delta.down == -1

    rotated = b.rotate_by_angle(90)
    assert round(rotated.north, 3) == -1
    assert round(rotated.east, 3) == -1
    assert rotated.down == 1
    
    rotated = b.rotate_by_angle(180)
    assert round(rotated.north, 3) == -1
    assert round(rotated.east, 3) == 1
    assert rotated.down == 1
    
    rotated = b.rotate_by_angle(45)
    assert round(rotated.north, 3) == 0
    assert round(rotated.east, 3) == -1.414
    assert rotated.down == 1
    
    rotated = b.rotate_by_angle(-45)
    assert round(rotated.north, 3) == 1.414
    assert round(rotated.east, 3) == 0
    assert rotated.down == 1
    
    a = VectorNED(3, 0, 0)
    a = a.norm()
    assert(round(a.north, 3)) == 1
    
    a = VectorNED(0, -3, 0)
    a = a.norm()
    assert(round(a.east, 3)) == -1
    
    a = VectorNED(1, 1, 0)
    a = a.norm()
    assert(round(a.north, 3)) == 0.707
    assert(round(a.east, 3)) == 0.707
    
    a = VectorNED(1, 1, 0)
    a = a * 2
    assert(round(a.north, 3)) == 2
    assert(round(a.east, 3)) == 2


def test_geofence_checker():
    ###################
    ### ROVER TESTS ###
    ###################
    checker = SafetyChecker("geofence_config_rover_test.yaml")
    # valid lat and long
    (valid_waypoint, error_msg) = checker.validate_waypoint(35.72863271013824, -78.69887262268281, 35.72723646570659, -78.69938351360769)
    assert(valid_waypoint)

    # next lat and long outside geofence
    (valid_waypoint, error_msg) = checker.validate_waypoint(35.72863271013824, -78.69887262268281, 35.72747396210598, -78.70120089855675)
    assert(not valid_waypoint)

    # next location crosses geofence
    (valid_waypoint, error_msg) = checker.validate_waypoint(35.72550320983546, -78.7005851431933, 35.724562532570005, -78.69970537867525)
    assert(not valid_waypoint)
    
    # same as above but opposite direction
    (valid_waypoint, error_msg) = checker.validate_waypoint(35.724562532570005, -78.69970537867525, 35.72550320983546, -78.7005851431933)
    assert(not valid_waypoint)

    # next location inside no-go
    (valid_waypoint, error_msg) = checker.validate_waypoint(35.72863271013824, -78.69887262268281, 35.725490953416376, -78.69734105606496, 20)
    assert(not valid_waypoint)

    # next location requires crossing no-go
    (valid_waypoint, error_msg) = checker.validate_waypoint(35.72863271013824, -78.69887262268281, 35.723229887405175, -78.69547821737531, 20)
    assert(not valid_waypoint)
    
    # same as above but opposite direction
    (valid_waypoint, error_msg) = checker.validate_waypoint(35.723229887405175, -78.69547821737531, 35.72863271013824, -78.69887262268281, 20)
    assert(not valid_waypoint)
    
    ####################
    ### COPTER TESTS ###
    ####################
    checker = SafetyChecker("geofence_config_copter_test.yaml")
    (valid_waypoint, error_msg) = checker.validate_waypoint(35.72863271013824, -78.69887262268281, 35.728481947918695, -78.69619704938518, 20)
    assert(valid_waypoint)
    # valid lat and long but invalid alt
    (valid_waypoint, error_msg) = checker.validate_waypoint(35.72863271013824, -78.69887262268281, 35.728481947918695, -78.69619704938518, 10)
    assert(not valid_waypoint)
    # invalid lat and long
    (valid_waypoint, error_msg) = checker.validate_waypoint(35.72863271013824, -78.69887262268281, 35.72747396210598, -78.70120089855675, 20)
    assert(not valid_waypoint)
    
    # next location crosses geofence
    (valid_waypoint, error_msg) = checker.validate_waypoint(35.72550320983546, -78.7005851431933, 35.724562532570005, -78.69970537867525)
    assert(not valid_waypoint)
    
    # same as above but opposite direction
    (valid_waypoint, error_msg) = checker.validate_waypoint(35.724562532570005, -78.69970537867525, 35.72550320983546, -78.7005851431933)
    assert(not valid_waypoint)

    # next location inside no-go
    (valid_waypoint, error_msg) = checker.validate_waypoint(35.72863271013824, -78.69887262268281, 35.725490953416376, -78.69734105606496, 20)
    assert(not valid_waypoint)

    # next location requires crossing no-go
    (valid_waypoint, error_msg) = checker.validate_waypoint(35.72863271013824, -78.69887262268281, 35.723229887405175, -78.69547821737531, 20)
    assert(not valid_waypoint)
    
    # same as above but opposite direction
    (valid_waypoint, error_msg) = checker.validate_waypoint(35.723229887405175, -78.69547821737531, 35.72863271013824, -78.69887262268281, 20)
    assert(not valid_waypoint)



if __name__ == "__main__":
    test_geofence_checker()