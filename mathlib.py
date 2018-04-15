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


class Spline():
    def __init__(self, waypoints: List):
        assert len(waypoints) >= 2
        self.waypoints = waypoints[:]
        self.length = 0
        self.parts = []
        self.reticulate()

    def reticulate(self):
        raise NotImplementedError

    def get_part(self, t: float):
        for part in self.parts:
            if part.t_begin <= t <= part.t_end:
                _part = part
                break
        else:
            _part = self.parts[-1]
        return _part

    def get_point(self, t: float):
        if t > 1:
            return self.get_point(1) + Vector2(t/self.length, t*self.get_slope(1) / self.length)
        _part = self.get_part(t)
        return Vector2(_part.get_x(t), _part.compute(t))

    def get_slope(self, t: float):
        # assert 0 <= t <= 1
        _part = self.get_part(t)
        return _part.slope(t)

    def get_unit_tangent_vector(self, t: float):
        _part = self.get_part(t)
        return Vector2(1, _part.slope(t)).normalized()


class CubicSpline(Spline):
    def __init__(self, waypoints: List):
        super().__init__(waypoints)

    def reticulate(self):
        waypoints = self.waypoints
        curves = []
        lengths = []
        for n in range(len(waypoints) - 1):
            waypoint = waypoints[n]
            next_waypoint = waypoints[n + 1]

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

            def get_cubic_system(x0, y0, x1, y1, t0, t1):
                return np.mat([Polynomial.get_row_for_degree(x0, 3),
                               Polynomial.get_row_for_degree(x1, 3),
                               Polynomial.get_slope_row_for_degree(x0, 3),
                               Polynomial.get_slope_row_for_degree(x1, 3)
                               ]), np.mat([[y0], [y1], [t0], [t1]])

            A, b = get_cubic_system(x0, y0, x1, y1, t0, t1)
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


class ComboSpline(Spline):
    def __init__(self, waypoints: List):
        super().__init__(waypoints)

    def reticulate(self):
        def get_quartic_system(x0, y0, x1, y1, t0, t1, k0, k1):
            return np.mat([Polynomial.get_row_for_degree(x0, 4),
                           Polynomial.get_row_for_degree(x1, 4),
                           Polynomial.get_slope_row_for_degree(x0, 4),
                           Polynomial.get_slope_row_for_degree(x1, 4),
                           Polynomial.get_curvature_row_for_degree(x0, 4),
                           ]), np.mat([[y0], [y1], [t0], [t1], [k0]])

        def get_quintic_system(x0, y0, x1, y1, t0, t1, k0, k1) -> Tuple[np.mat, np.mat]:
            return np.mat([Polynomial.get_row_for_degree(x0, 5),
                           Polynomial.get_row_for_degree(x1, 5),
                           Polynomial.get_slope_row_for_degree(x0, 5),
                           Polynomial.get_slope_row_for_degree(x1, 5),
                           Polynomial.get_curvature_row_for_degree(x0, 5),
                           Polynomial.get_curvature_row_for_degree(x1, 5),
                           ]), np.mat([[y0], [y1], [t0], [t1], [k0], [k1]])
        
        waypoints = self.waypoints
        curves = []
        lengths = []
        for n in range(len(waypoints) - 1):
            waypoint = waypoints[n]
            next_waypoint = waypoints[n+1]
            quintic_flag = n+1 == len(waypoints) - 1

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

            get_system = get_quintic_system if quintic_flag else get_quartic_system
            A, b = get_system(x0, y0, x1, y1, t0, t1, k0, k1)
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

