import math
from typing import Union, List, Tuple
import numpy as np


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


def approximate_curve(x0, y0, xs, s, k):
    M = np.mat([[x0 ** 2, x0, 1], [2 * xs, 1, 0], [2, 0, 0]])
    b = np.mat([[y0], [s], [k]])
    coeff = np.linalg.solve(M, b)
    return polynomial_from_parameters(coeff)


class Polynomial:
    def __init__(self):
        pass

    def compute(self, x: float) -> float:
        raise NotImplementedError

    def slope(self, x: float) -> float:
        raise NotImplementedError

    def second_deriv(self, x: float) -> float:
        raise NotImplementedError

    @staticmethod
    def get_row(x: float) -> List[float]:
        raise NotImplementedError

    @staticmethod
    def get_slope_row(x: float) -> List[float]:
        raise NotImplementedError

    @staticmethod
    def get_curvature_row(x: float) -> List[float]:
        raise NotImplementedError

    @staticmethod
    def get_system(x0, y0, x1, y1, t0, t1, k0, k1) -> Tuple[np.mat, np.mat]:
        raise NotImplementedError

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


class QuadraticPolynomial(Polynomial):
    def __init__(self, A, B, C):
        super().__init__()
        self.A = A
        self.B = B
        self.C = C

    def compute(self, x) -> float:
        return self.A * x**2 + \
               self.B * x**1 + \
               self.C

    def slope(self, x) -> float:
        return 2 * self.A * x**1 + \
               self.B

    def second_deriv(self, x) -> float:
        return 2 * self.A

    @staticmethod
    def get_row(x: float) -> List[float]:
        return [x**2, x**1, x**0]

    @staticmethod
    def get_slope_row(x: float) -> List[float]:
        return [2 * x ** 1, x ** 0, 0]

    @staticmethod
    def get_curvature_row(x: float) -> List[float]:
        return [2 * x ** 0, 0, 0]

    def __str__(self):
        return f"Quadratic {self.A} {self.B} {self.C}"


class QuarticPolynomial(Polynomial):
    def __init__(self, A, B, C, D, E):
        super().__init__()
        self.A = A
        self.B = B
        self.C = C
        self.D = D
        self.E = E

    def compute(self, x) -> float:
        return self.A * x**4 + \
               self.B * x**3 + \
               self.C * x**2 + \
               self.D * x**1 + \
               self.E

    def slope(self, x) -> float:
        return 4 * self.A * x**3 + \
               3 * self.B * x**2 + \
               2 * self.C * x**1 + \
               self.D

    def second_deriv(self, x) -> float:
        return 12 * self.A * x**2 + 6 * self.B * x + 2 * self.C

    def curvature(self, x):
        return self.second_deriv(x) / (1 + self.slope(x)**2)**(3/2)

    @staticmethod
    def get_row(x: float) -> List[float]:
        return [x**4, x**3, x**2, x**1, x**0]

    @staticmethod
    def get_slope_row(x: float) -> List[float]:
        return [4 * x ** 3, 3 * x ** 2, 2 * x ** 1, x ** 0, 0]

    @staticmethod
    def get_curvature_row(x: float) -> List[float]:
        return [12 * x ** 2, 6 * x ** 1, 2 * x ** 0, 0, 0]

    @staticmethod
    def get_system(x0, y0, x1, y1, t0, t1, k0, k1):
        return np.mat([QuarticPolynomial.get_row(x0),
                       QuarticPolynomial.get_row(x1),
                       QuarticPolynomial.get_slope_row(x0),
                       QuarticPolynomial.get_slope_row(x1),
                       QuarticPolynomial.get_curvature_row(x0),
                       ]), np.mat([[y0], [y1], [t0], [t1], [k0]])

    def __str__(self):
        return f"Quartic {self.A} {self.B} {self.C} {self.D} {self.E}"


class QuinticPolynomial(Polynomial):
    def __init__(self, A, B, C, D, E, F):
        super().__init__()
        self.A = A
        self.B = B
        self.C = C
        self.D = D
        self.E = E
        self.F = F

    def compute(self, x):
        return self.A * x ** 5 + \
               self.B * x ** 4 + \
               self.C * x ** 3 + \
               self.D * x ** 2 + \
               self.E * x ** 1 + \
               self.F

    def slope(self, x):
        return 5 * self.A * x ** 4 + \
               4 * self.B * x ** 3 + \
               3 * self.C * x ** 2 + \
               2 * self.D * x ** 1 + \
               self.E

    def second_deriv(self, x) :
        return 20 * self.A * x**3 + 12 * self.B * x**2 + 6 * self.C * x + self.D

    @staticmethod
    def get_row(x: float) -> List[float]:
        return [x ** 5, x ** 4, x ** 3, x ** 2, x ** 1, x ** 0]

    @staticmethod
    def get_slope_row(x: float) -> List[float]:
        return [5 * x ** 4, 4 * x ** 3, 3 * x ** 2, 2 * x ** 1, x ** 0, 0]

    @staticmethod
    def get_curvature_row(x: float) -> List[float]:
        return [20 * x **3, 12 * x ** 2, 6 * x ** 1, 2 * x ** 0, 0, 0]

    @staticmethod
    def get_system(x0, y0, x1, y1, t0, t1, k0, k1) -> Tuple[np.mat, np.mat]:
        return np.mat([QuinticPolynomial.get_row(x0),
                       QuinticPolynomial.get_row(x1),
                       QuinticPolynomial.get_slope_row(x0),
                       QuinticPolynomial.get_slope_row(x1),
                       QuinticPolynomial.get_curvature_row(x0),
                       QuinticPolynomial.get_curvature_row(x1),
                       ]), np.mat([[y0], [y1], [t0], [t1], [k0], [k1]])

    def __str__(self):
        return f"Quintic {self.A} {self.B} {self.C} {self.D} {self.E} {self.F}"


def polynomial_from_parameters(parameters: np.ndarray) -> Polynomial:
    parameters = list(parameters.flatten().tolist())[0]  # ew ew ew
    if len(parameters) == 3:
        return QuadraticPolynomial(parameters[0], parameters[1], parameters[2])
    elif len(parameters) == 5:
        return QuarticPolynomial(parameters[0], parameters[1], parameters[2], parameters[3], parameters[4])
    elif len(parameters) == 6:
        return QuinticPolynomial(parameters[0], parameters[1], parameters[2], parameters[3], parameters[4],
                                 parameters[5])
    else:
        raise ValueError("Only quadratic, quintic and quartic polynomials expected")


class SplinePart:
    def __init__(self, P: Polynomial, x0: float, x1: float, t_begin: float, t_end: float):
        self.curve = P
        self.begin_x = x0
        self.end_x = x1
        self.length = P.length(x0, x1)
        self.t_begin = t_begin
        self.t_end = t_end

    def get_x(self, t) -> float:
        return self.begin_x + (self.end_x - self.begin_x) * (t - self.t_begin) / (self.t_end - self.t_begin)

    def compute(self, t: float) -> float:
        return self.curve.compute(self.get_x(t))

    def slope(self, t: float) -> float:
        return self.curve.slope(self.get_x(t))

    def curvature(self, t: float) -> float:
        return self.curve.second_deriv(self.get_x(t))


class ComboSpline:
    def __init__(self, waypoints: List):
        assert len(waypoints) >= 2

        self.waypoints = waypoints[:]

        lengths = []
        curves = []

        for n in range(len(waypoints) - 1):
            waypoint = waypoints[n]
            next_waypoint = waypoints[n+1]
            quintic_flag = n+1 == len(waypoints) - 1
            spline_type = QuinticPolynomial if quintic_flag else QuarticPolynomial

            # First waypoint has zero curvature to begin
            if n == 0:
                k0 = 0
            else:
                k0 = curves[-1].second_deriv(waypoint.x)
            k1 = 0  # zero curvature at the end, only used on last spline

            x0 = waypoint.x
            y0 = waypoint.y
            x1 = next_waypoint.x
            y1 = next_waypoint.y
            t0 = math.tan(waypoint.heading)
            t1 = math.tan(next_waypoint.heading)

            cap_tangent = 1e2
            if abs(t0) > cap_tangent:
                print(f"Capping large angle at knot {n}")
                t0 = math.copysign(cap_tangent, t0)
            if abs(t1) > cap_tangent:
                t1 = math.copysign(cap_tangent, t1)
                print(f"Capping large angle at knot {n+1}")

            A, b = spline_type.get_system(x0, y0, x1, y1, t0, t1, k0, k1)
            curve_constants = np.linalg.solve(A, b)
            P = polynomial_from_parameters(curve_constants)
            curves.append(P)
            lengths.append(P.length(waypoint.x, next_waypoint.x))

        self.length = sum(lengths)
        for i in range(len(lengths)):
            lengths[i] /= self.length

        self.parts = []
        t_accum = 0
        for i in range(len(curves)):
            wp = waypoints[i]
            n_wp = waypoints[i + 1]
            x0 = wp.x
            x1 = n_wp.x
            t_begin = t_accum
            t_end = t_accum + lengths[i]
            t_accum = t_end
            part = SplinePart(curves[i], x0, x1, t_begin, t_end)
            assert abs(part.get_x(t_begin) - x0) < 0.001
            assert part.get_x(t_end) == x1

            self.parts.append(part)

    def get_part(self, t: float):
        for part in self.parts:
            if part.t_begin <= t <= part.t_end:
                _part = part
                break
        else:
            _part = self.parts[-1]
        return _part

    def get_point(self, t: float):
        # assert 0 <= t <= 1
        _part = self.get_part(t)
        return Vector2(_part.get_x(t), _part.compute(t))

    def get_unit_tangent_vector(self, t: float):
        _part = self.get_part(t)
        return Vector2(1, _part.slope(t)).normalized()


def minimize_diff(P1: QuadraticPolynomial, P2: QuadraticPolynomial):
    minimize = QuadraticPolynomial(abs(P1.A - P2.A), abs(P1.B - P2.B), abs(P1.C - P2.C))
    if minimize.A == 0:
        raise ValueError("Polynomials are the same")
    x = minimize.B / (2 * minimize.A)
    return x

