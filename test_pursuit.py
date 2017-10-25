import math

import pursuit
from mathlib import Vector2, Line


def eq_epsilon(a, b, epsilon=1e-4):
    return abs(a - b) < epsilon


def test_distance():
    p1 = Vector2(0, 0)
    p2 = Vector2(1, 0)
    p3 = Vector2(3, 4)

    assert p1.distance(p2) == 1
    assert p1.distance(p3) == 5


def test_translate():
    pose = pursuit.Pose(1, 1, 0)
    p1 = Vector2(2, 2)
    p2 = Vector2(2, 1)
    pose2 = pursuit.Pose(1, 1, math.pi / 4)
    assert eq_epsilon(p1.translated(pose), Vector2(1, 1))
    assert eq_epsilon(p2.translated(pose), Vector2(1, 0))
    assert eq_epsilon(p1.translated(pose2), Vector2(2 ** 0.5, 0))

    assert eq_epsilon(p1.translated(pose2).inv_translate(pose2), p1)
    assert eq_epsilon(p2.translated(pose2).inv_translate(pose2), p2)
    assert eq_epsilon(p1.translated(pose).inv_translate(pose), p1)
    assert eq_epsilon(p2.translated(pose).inv_translate(pose), p2)


def test_lines():
    p1 = Vector2(0, 0)
    p2 = Vector2(1, 1)
    L = Line(p1, p2)
    assert eq_epsilon(L.r(0), p1)
    assert eq_epsilon(L.invert(p1), 0)
    assert eq_epsilon(L.r(2**0.5), p2)
    assert eq_epsilon(L.invert(p2), 2**0.5)