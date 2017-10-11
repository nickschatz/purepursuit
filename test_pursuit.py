import math

import pursuit


def eq_epsilon(a, b, epsilon=1e-4):
    return abs(a - b) < epsilon


def test_distance():
    p1 = pursuit.Waypoint(0, 0)
    p2 = pursuit.Waypoint(1, 0)
    p3 = pursuit.Waypoint(3, 4)

    assert p1.distance(p2) == 1
    assert p1.distance(p3) == 5


def test_line_circ_intercept():
    p1 = pursuit.Waypoint(0, 1)
    p2 = pursuit.Waypoint(1, 0)

    i1, i2 = pursuit.line_circ_intercepts(p1, p2, 1)
    assert eq_epsilon(i1, p1) or eq_epsilon(i1, p2)
    assert eq_epsilon(i2, p2) or eq_epsilon(i2, p1)

    p3 = pursuit.Waypoint(-1, 0)
    i3, i4 = pursuit.line_circ_intercepts(p3, p2, 1)
    assert eq_epsilon(i3, p3) or eq_epsilon(i3, p2)
    assert eq_epsilon(i4, p2) or eq_epsilon(i4, p3)

def test_translate():
    pose = pursuit.Pose(1, 1, 0)
    p1 = pursuit.Waypoint(2, 2)
    p2 = pursuit.Waypoint(2, 1)
    pose2 = pursuit.Pose(1, 1, math.pi / 4)
    assert eq_epsilon(p1.translated(pose), pursuit.Waypoint(1, 1))
    assert eq_epsilon(p2.translated(pose), pursuit.Waypoint(1, 0))
    assert eq_epsilon(p1.translated(pose2), pursuit.Waypoint(0, 2**0.5))

    assert eq_epsilon(p1.translated(pose2).inv_translate(pose2), p1)
    assert eq_epsilon(p2.translated(pose2).inv_translate(pose2), p2)
    assert eq_epsilon(p1.translated(pose).inv_translate(pose), p1)
    assert eq_epsilon(p2.translated(pose).inv_translate(pose), p2)

