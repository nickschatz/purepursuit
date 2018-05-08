import math

from py_pursuit_pathing.mathlib import Arc, Vector2, normalize_angle


def epsilon_eq(a, b, epsilon = 1e-4):
    return abs(a-b) < epsilon


def test_normalize():
    assert epsilon_eq(normalize_angle(-math.pi/2), 3*math.pi/2)


def test_on_arc():
    arc = Arc(Vector2(0, 0), start_angle=math.pi/2, end_angle=0, radius=1)
    arc2 = Arc(Vector2(0, 0), start_angle=0, end_angle=math.pi/2, radius=1)
    arc3 = Arc(Vector2(0, 0), start_angle=-math.pi/2, end_angle=math.pi/2, radius=1)

    assert arc.angle_in(math.pi/4)
    assert arc2.angle_in(math.pi/4)
    assert arc3.angle_in(0)


def test_points():
    arc = Arc(Vector2(0, 0), start_angle=math.pi/2, end_angle=0, radius=1)
    arc2 = Arc(Vector2(0, 0), start_angle=0, end_angle=math.pi / 2, radius=1)
    epsilon = 1e-4
    assert arc.r(0).distance(Vector2(0, 1)) < epsilon
    assert arc2.r(0).distance(Vector2(1, 0)) < epsilon
    assert arc.r(1).distance(Vector2(1, 0)) < epsilon
    assert arc2.r(1).distance(Vector2(0, 1)) < epsilon

    arc3 = Arc(Vector2(0,0), start_angle=2*math.pi, end_angle=3*math.pi, radius=1)
    assert arc3.r(0).distance(Vector2(1,0)) < epsilon
    assert arc3.r(1).distance(Vector2(-1, 0)) < epsilon