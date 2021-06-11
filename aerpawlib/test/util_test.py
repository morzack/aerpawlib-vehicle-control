# test coordinate logic
# this is mainly to make sure that the math is correct

from aerpawlib.util import Coordinate, VectorNED


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
