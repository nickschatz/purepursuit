import math
from typing import Union


class Vector2:
    """
    Waypoint for path planning. x,y are in world coordinates
    """

    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y

    def translated(self, pose):
        dx = self.x - pose.x
        dy = self.y - pose.y
        dxp = dx * math.cos(pose.heading) - dy * math.sin(pose.heading)
        dyp = dx * math.sin(pose.heading) + dy * math.cos(pose.heading)
        return Vector2(dxp, dyp)

    def inv_translate(self, pose):
        dxp = self.x * math.cos(pose.heading) + self.y * math.sin(pose.heading)
        dyp = self.x * -math.sin(pose.heading) + self.y * math.cos(pose.heading)
        return Vector2(dxp + pose.x, dyp + pose.y)

    def distance(self, point):
        return ((point.x - self.x)**2 + (point.y - self.y)**2)**0.5

    def normalized(self):
        magn = abs(self)
        return Vector2(self.x / magn, self.y / magn)

    def __mul__(self, other):
        if type(other) == Vector2:
            return self.x * other.x + self.y * other.y
        else:
            return Vector2(self.x * other, self.y * other)

    def __repr__(self):
        return "Vector({}, {})".format(self.x, self.y)

    def __sub__(self, other):
        assert type(other) == Vector2
        return Vector2(self.x - other.x, self.y - other.y)

    def __add__(self, other):
        assert type(other) == Vector2
        return Vector2(self.x + other.x, self.y + other.y)

    def __abs__(self):
        return self.distance(Vector2(0, 0))


class Line:
    def __init__(self, point1: Vector2, point2: Vector2):
        self.slope = Vector2(point2.x - point1.x, point2.y - point1.y).normalized()
        self.intersect = point1

    def projected_point(self, point: Vector2):
        pt = point - self.intersect
        a_scalar = pt * self.slope
        return self.slope * a_scalar + self.intersect

    def invert(self, point: Vector2):
        tr_point = point - self.intersect
        if self.slope.x == 0:
            return tr_point.y / self.slope.y
        elif self.slope.y == 0:
            return tr_point.x / self.slope.x
        return (tr_point.x / self.slope.x + tr_point.y / self.slope.y) / 2

    def r(self, t):
        return self.intersect + self.slope * t

    def __repr__(self):
        return "Line(t<{}, {}> + <{}, {}>".format(self.slope.x, self.slope.y, self.intersect.x, self.intersect.y)
