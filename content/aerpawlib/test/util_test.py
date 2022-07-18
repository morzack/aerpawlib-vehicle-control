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
    
