import math
from numbers import Number
from typing import Union, List, Tuple, Any
import numpy as np


def normalize_angle(angle: float):
    ang = angle % (2*math.pi)
    if ang < 0:
        return ang + 2*math.pi
    return ang


def to_heading(angle: float) -> float:
    n = normalize_angle(angle)
    if n > math.pi:
        return n - 2*math.pi
    return n


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
        return self.sq_dist(point)**0.5

    def sq_dist(self, point):
        return (point.x - self.x) ** 2 + (point.y - self.y) ** 2

    def normalized(self):
        magn = abs(self)
        return Vector2(self.x / magn, self.y / magn)

    def __mul__(self, other):
        if type(other) in (float, int, np.float64):
            return Vector2(self.x * other, self.y * other)
        return self.x * other.x + self.y * other.y

    def __rmul__(self, other):
        return self * other

    def __truediv__(self, other: Union[float, int]):
        assert type(other) in (float, int)
        return Vector2(self.x / other, self.y / other)

    def __repr__(self):
        return "Vector({}, {})".format(self.x, self.y)

    def __sub__(self, other):
        # quack quack
        return Vector2(self.x - other.x, self.y - other.y)

    def __add__(self, other):
        return Vector2(self.x + other.x, self.y + other.y)

    def __abs__(self):
        return self.distance(Vector2(0, 0))

    def __copy__(self):
        return Vector2(self.x, self.y)

    def __neg__(self):
        return self * -1

    def cross(self, other) -> float:
        return self.x * other.y - self.y * other.x

    def angle(self):
        return math.atan2(self.y, self.x)


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

    def r(self, t: float) -> Vector2:
        return self.intersect + self.slope * (t * self.max_t)

    def unit_tangent_vector(self, t: float) -> Vector2:
        return self.slope

    def arc_length(self):
        return self.max_t

    def on_line(self, point: Vector2, epsilon=1e-3):

        tr_point = point - self.intersect
        if self.slope.x == 0:
            return abs(tr_point.x) < epsilon
        elif self.slope.y == 0:
            return abs(tr_point.y) < epsilon
        return abs(tr_point.x / self.slope.x - tr_point.y / self.slope.y) < epsilon

    def __repr__(self):
        return "Line(t<{}, {}> + <{}, {}>".format(self.slope.x, self.slope.y, self.intersect.x, self.intersect.y)


def approximate_curve(x0, y0, xs, s, k):
    M = np.mat([[x0 ** 2, x0, 1], [2 * xs, 1, 0], [2, 0, 0]])
    b = np.mat([[y0], [s], [k]])
    coeff = np.linalg.solve(M, b)
    return polynomial_from_parameters(coeff)


class Polynomial:
    def __init__(self, coefficients: List[float]):
        self.coefficients = coefficients

    def compute(self, x: float) -> float:
        sum = 0
        for i, coeff in enumerate(reversed(self.coefficients)):
            sum += coeff * (x**i)
        return sum

    def slope(self, x: float) -> float:
        sum = 0
        for i, coeff in enumerate(reversed(self.coefficients)):
            if i == 0:
                continue
            sum += i * coeff * (x ** (i-1))
        return sum

    def second_deriv(self, x: float) -> float:
        sum = 0
        for i, coeff in enumerate(reversed(self.coefficients)):
            if i == 0 or i == 1:
                continue
            sum += i * (i - 1) * coeff * (x ** (i - 2))
        return sum
    
    def get_degree(self) -> int:
        return len(self.coefficients) - 1

    @staticmethod
    def get_row_for_degree(x: float, degree: int) -> List[float]:
        return [x**i for i in range(degree, -1, -1)]

    @staticmethod
    def get_slope_row_for_degree(x: float, degree: int) -> List[float]:
        return [i * x**(i-1) for i in range(degree, 0, -1)] + [0]

    @staticmethod
    def get_curvature_row_for_degree(x: float, degree: int) -> List[float]:
        return [i * (i - 1) * x ** (i - 2) for i in range(degree, 1, -1)] + [0, 0]

    def __str__(self):
        return f"P_{self.get_degree()} {self.coefficients}"

    def length(self, x0: float, x1: float) -> float:
        accum = 0
        dx = 0.001
        x = float(x0)
        while x < x1:
            slope = float(self.slope(x))
            accum += (1 + slope**2)**(1/2) * dx
            x += dx
        return accum

    def get_local_quadratic_approximation(self, x: float):
        return approximate_curve(x, self.compute(x), x, self.slope(x), self.second_deriv(x))


def polynomial_from_parameters(parameters: np.ndarray) -> Polynomial:
    parameters = list(parameters.flatten().tolist())[0]  # ew ew ew
    return Polynomial(parameters)

def sgn(x):
    return 0 if x == 0 else x/abs(x)

class Arc:
    def __init__(self, center: Vector2, radius: float, start_angle: float, end_angle: float,
                 direction: int = 1):
        self.center: Vector2 = center
        self.radius: float = radius
        self.start_angle: float = start_angle
        self.end_angle: float = end_angle

        self._cache = {}

    def r(self, t: float) -> Vector2:
        if t in self._cache:
            return self._cache[t]
        angle = t * (self.end_angle - self.start_angle) + self.start_angle
        r_ = self.center + Vector2(math.cos(angle), math.sin(angle)) * self.radius
        self._cache[t] = r_
        return r_

    def unit_tangent_vector(self, t: float) -> Vector2:
        angle = t * (self.end_angle - self.start_angle) + self.start_angle
        return Vector2(math.sin(angle), math.cos(angle)).normalized()

    def arc_length(self):
        return self.radius * abs(self.end_angle - self.start_angle)

    def angle_in(self, angle: float) -> bool:
        return 0 <= self.invert(angle) <= 1

    def invert(self, angle: float) -> float:
        if self.end_angle - self.start_angle == 0:
            return 0
        return (angle - self.start_angle) / (self.end_angle - self.start_angle)

    def project(self, point: Vector2):
        angle = (self.center - point).angle()
        opp_angle = angle + math.pi
        neg_angle = angle - math.pi
        # assert 0 <= opp_angle <= 2 * math.pi
        if self.angle_in(angle):
            return self.invert(angle)
        elif self.angle_in(opp_angle):
            return self.invert(opp_angle)
        elif self.angle_in(neg_angle):
            return self.invert(neg_angle)
        raise ValueError

    def __repr__(self):
        return f"Arc(center={self.center}, radius={self.radius}, start_angle={self.start_angle}, end_angle={self.end_angle}"
