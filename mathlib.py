import math
from typing import Union, List
import numpy as np

import pursuit


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
        c = math.cos(pose.heading)
        s = math.sin(pose.heading)
        dxp = dx * c + dy * s
        dyp = dx * -s + dy * c
        return Vector2(dxp, dyp)

    def inv_translate(self, pose):
        c = math.cos(pose.heading)
        s = math.sin(pose.heading)
        dxp = self.x * c - self.y * s
        dyp = self.x * s + self.y * c
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
        assert type(other) == Vector2 or type(other) == pursuit.Pose
        return Vector2(self.x + other.x, self.y + other.y)

    def __abs__(self):
        return self.distance(Vector2(0, 0))

    def __copy__(self):
        return Vector2(self.x, self.y)

    def __neg__(self):
        return self * -1


class LineSegment:
    def __init__(self, point1: Vector2, point2: Vector2):
        d = Vector2(point2.x - point1.x, point2.y - point1.y)
        self.slope = d.normalized()
        self.max_t = abs(d)
        self.intersect = point1
        self.point1 = point1
        self.point2 = point2

    def plot(self, plt, resolution=0.1):
        t0 = 0
        t1 = self.max_t
        i = t0
        xx = []
        yy = []
        while i < t1:
            p = self.r(i)
            xx += [p.x]
            yy += [p.y]
            i += resolution
        plt.plot(xx, yy)

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

    def on_line(self, point: Vector2, epsilon=1e-3):

        tr_point = point - self.intersect
        if self.slope.x == 0:
            return abs(tr_point.x) < epsilon
        elif self.slope.y == 0:
            return abs(tr_point.y) < epsilon
        return abs(tr_point.x / self.slope.x - tr_point.y / self.slope.y) < epsilon

    def in_segment(self, t: float):
        return 0 <= t <= self.max_t

    def __repr__(self):
        return "Line(t<{}, {}> + <{}, {}>".format(self.slope.x, self.slope.y, self.intersect.x, self.intersect.y)


class CubicSpline:
    def __init__(self, waypoints: List[pursuit.Pose]):
        assert len(waypoints) >= 3
        self.curves = []

        for n in range(len(waypoints) - 2):
            waypoint = waypoints[n]
            next_waypoint = waypoints[n+1]
            # Last polynomial, needs to be quintic
            if n+1 == len(waypoints) - 1:
                pass
            else:
                # First waypoint has zero curvature to begin
                if n == 0:
                    k0 = 0
                else:
                    k0 = self.curves[-1].end_curvature()
                x0 = waypoint.x
                y0 = waypoint.y
                x1 = next_waypoint.x
                y1 = next_waypoint.y
                t0 = math.tan(waypoint.heading)
                t1 = math.tan(next_waypoint.heading)

                A = np.mat([[x0**4, x0**3, x0**2, x0, 1],
                            [x1 ** 4, x1 ** 3, x1 ** 2, x1, 1],
                            [4 * x0**3, 3 * x0**2, 2 * x0, 1, 0],
                            [4 * x1**3, 3 * x1**2, 2 * x1, 1, 0],
                            [12 * x0**2, 6 * x0, 2, 0, 0]])
                b = np.mat([[y0], [y1], [t0], [t1], [k0]])
                curve_constants = np.


