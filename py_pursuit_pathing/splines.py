import math
import typing
from typing import List, Tuple, Dict, Union

import numpy as np

from py_pursuit_pathing import mathlib
from py_pursuit_pathing.mathlib import Polynomial, Vector2, polynomial_from_parameters, LineSegment, Arc
from py_pursuit_pathing.pose import Pose


class SplinePart:
    def get_point(self, t: float) -> Vector2:
        raise NotImplementedError

    def get_x(self, t: float) -> float:
        raise NotImplementedError

    def get_y(self, t: float) -> float:
        raise NotImplementedError

    def length(self, t: float) -> float:
        raise NotImplementedError

    def get_unit_tangent_vector(self, t: float) -> Vector2:
        raise NotImplementedError

    def project(self, point: Vector2):
        raise NotImplementedError


class CurvePart(SplinePart):
    def __init__(self, P: Polynomial, x0: float, x1: float, t_begin: float, t_end: float, offset: Pose):
        self.curve = P
        self.begin_x = x0
        self.end_x = x1
        self.total_arc_length = P.length(x0, x1)
        self.t_begin = t_begin
        self.t_end = t_end
        self.offset = offset

    def get_point(self, t: float) -> Vector2:
        return (Vector2(self._get_x(t), self._get_y(t))).inv_translate(self.offset)

    def get_x(self, t: float) -> float:
        return self.get_point(t).x

    def get_y(self, t: float) -> float:
        return self.get_point(t).y

    def _get_x(self, t: float) -> float:
        return self.end_x * (t - self.t_begin) / (self.t_end - self.t_begin)

    def _get_y(self, t: float) -> float:
        return self.curve.compute(self._get_x(t))

    def length(self, t: float) -> float:
        return self.curve.length(self._get_x(self.t_begin), self._get_x(t))

    def get_unit_tangent_vector(self, t: float) -> Vector2:
        angle = math.atan(self.curve.slope(self._get_x(t)))
        real_angle = angle - self.offset.heading
        return Vector2(math.cos(real_angle), math.sin(real_angle)).normalized()

    def curvature(self, t: float) -> float:
        return self.curve.second_deriv(self.get_x(t))


class Spline():
    def __init__(self, waypoints: List[Pose]):
        assert len(waypoints) >= 2
        self.waypoints: List[Pose] = waypoints[:]
        self.length: float = 0
        self.parts: List[CurvePart] = []
        self.reticulate()
        self._part_map: Dict[float, CurvePart] = {}
        self._point_cache: Dict[float, Vector2] = {}

    def reticulate(self):
        raise NotImplementedError

    def get_part(self, t: float):
        if t in self._part_map.keys():
            return self._part_map[t]
        for part in self.parts:
            if part.t_begin <= t <= part.t_end:
                _part = part
                break
        else:
            _part = self.parts[-1]
        self._part_map[t] = _part
        return _part

    def get_point(self, t: float):
        if t in self._point_cache:
            return self._point_cache[t]
        if t > 1:
            return self.get_point(1) + self.get_unit_tangent_vector(1) * (t / self.length)
        _part = self.get_part(t)
        pt = Vector2(_part.get_x(t), _part.get_y(t))
        self._point_cache[t] = pt
        return pt

    def get_unit_tangent_vector(self, t: float) -> Vector2:
        _part = self.get_part(t)
        return _part.get_unit_tangent_vector(t)

    def arc_length(self, t: float) -> float:
        length = 0
        for part in self.parts:
            if t >= part.t_end:
                length += part.total_arc_length
            if part.t_end > t > part.t_begin:
                length += part.length(t)
        return length

    def get_closest_t_to(self, point: Vector2):
        raise NotImplementedError


class LinearSpline(Spline):
    def __init__(self, waypoints: List[Pose]):
        super().__init__(waypoints)

    def reticulate(self):
        curves = []
        lengths = []
        for n in range(len(self.waypoints) - 1):
            waypoint = self.waypoints[n]
            next_waypoint = self.waypoints[n + 1].translated(waypoint)

            x0 = 0
            y0 = 0
            x1 = next_waypoint.x
            y1 = next_waypoint.y

            def get_linear_system(x0, y0, x1, y1):
                return np.mat([Polynomial.get_row_for_degree(x0, 1),
                               Polynomial.get_row_for_degree(x1, 1)
                               ]), np.mat([[y0], [y1]])

            A, b = get_linear_system(x0, y0, x1, y1)
            curve_constants = np.linalg.solve(A, b)
            P = polynomial_from_parameters(curve_constants)
            curves.append(P)
            lengths.append(P.length(x0, x1))

        self.length = sum(lengths)
        for i in range(len(lengths)):
            lengths[i] /= self.length

        self.parts = []
        t_accum = 0
        for i in range(len(curves)):
            wp = self.waypoints[i]
            n_wp = self.waypoints[i + 1].translated(wp)
            x0 = 0
            x1 = n_wp.x
            t_begin = t_accum
            t_end = t_accum + lengths[i]
            t_accum = t_end
            part = CurvePart(curves[i], x0, x1, t_begin, t_end, wp)

            self.parts.append(part)


class CubicSpline(Spline):
    def __init__(self, waypoints: List[Pose]):
        super().__init__(waypoints)

    def reticulate(self):
        curves = []
        lengths = []
        for n in range(len(self.waypoints) - 1):
            waypoint = self.waypoints[n]
            next_waypoint = self.waypoints[n + 1].translated(waypoint)

            x0 = 0
            y0 = 0
            x1 = next_waypoint.x
            y1 = next_waypoint.y
            t0 = math.tan(0)
            t1 = math.tan(next_waypoint.heading)

            cap_tangent = 1e2
            if abs(t1) > cap_tangent:
                t1 = math.copysign(cap_tangent, t1)
                print(f"Capping large angle at curve {n} -> {n+1}")

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
            lengths.append(P.length(x0, x1))

        self.length = sum(lengths)
        for i in range(len(lengths)):
            lengths[i] /= self.length

        self.parts = []
        t_accum = 0
        for i in range(len(curves)):
            wp = self.waypoints[i]
            n_wp = self.waypoints[i + 1].translated(wp)
            x0 = 0
            x1 = n_wp.x
            t_begin = t_accum
            t_end = t_accum + lengths[i]
            t_accum = t_end
            part = CurvePart(curves[i], x0, x1, t_begin, t_end, wp)

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
            next_waypoint = waypoints[n + 1].translated(waypoint)
            quintic_flag = n + 1 == len(waypoints) - 1

            # First waypoint has zero curvature to begin
            if n == 0:
                k0 = 0
            else:
                k0 = curves[-1].second_deriv(waypoint.x)
            k1 = 0  # zero curvature at the end, only used on last spline

            x0 = 0
            y0 = 0
            x1 = next_waypoint.x
            y1 = next_waypoint.y
            t0 = math.tan(0)
            t1 = math.tan(next_waypoint.heading)

            cap_tangent = 1e2
            if abs(t1) > cap_tangent:
                t1 = math.copysign(cap_tangent, t1)
                print(f"Capping large angle at knot {n+1}")

            get_system = get_quintic_system if quintic_flag else get_quartic_system
            A, b = get_system(x0, y0, x1, y1, t0, t1, k0, k1)
            curve_constants = np.linalg.solve(A, b)
            P = polynomial_from_parameters(curve_constants)
            curves.append(P)
            lengths.append(P.length(0, next_waypoint.x))

        self.length = sum(lengths)
        for i in range(len(lengths)):
            lengths[i] /= self.length

        self.parts = []
        t_accum = 0
        for i in range(len(curves)):
            wp = waypoints[i]
            n_wp = waypoints[i + 1].translated(wp)
            x0 = 0
            x1 = n_wp.x
            t_begin = t_accum
            t_end = t_accum + lengths[i]
            t_accum = t_end
            part = CurvePart(curves[i], x0, x1, t_begin, t_end, wp)

            self.parts.append(part)


class QuinticSpline(Spline):
    def __init__(self, waypoints: List):
        super().__init__(waypoints)

    def reticulate(self):
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
            next_waypoint = waypoints[n + 1].translated(waypoint)
            quintic_flag = n + 1 == len(waypoints) - 1

            # First waypoint has zero curvature to begin
            k0 = 0
            k1 = 0

            x0 = 0
            y0 = 0
            x1 = next_waypoint.x
            y1 = next_waypoint.y
            t0 = math.tan(0)
            t1 = math.tan(next_waypoint.heading)

            cap_tangent = 1e2
            if abs(t1) > cap_tangent:
                t1 = math.copysign(cap_tangent, t1)
                print(f"Capping large angle at knot {n+1}")

            A, b = get_quintic_system(x0, y0, x1, y1, t0, t1, k0, k1)
            curve_constants = np.linalg.solve(A, b)
            P = polynomial_from_parameters(curve_constants)
            curves.append(P)
            lengths.append(P.length(0, next_waypoint.x))

        self.length = sum(lengths)
        for i in range(len(lengths)):
            lengths[i] /= self.length

        self.parts = []
        t_accum = 0
        for i in range(len(curves)):
            wp = waypoints[i]
            n_wp = waypoints[i + 1].translated(wp)
            x0 = 0
            x1 = n_wp.x
            t_begin = t_accum
            t_end = t_accum + lengths[i]
            t_accum = t_end
            part = CurvePart(curves[i], x0, x1, t_begin, t_end, wp)

            self.parts.append(part)


class ArcPart(SplinePart):
    def _get_t(self, t):
        """
        Lerp t between 0 and 1
        :param t:
        :return:
        """
        return (t - self.t_begin) / (self.t_end - self.t_begin)

    def _inv_t(self, t):
        return t * (self.t_end - self.t_begin) + self.t_begin

    def get_point(self, t: float) -> Vector2:
        return self.part.r(self._get_t(t))

    def get_x(self, t: float) -> float:
        return self.get_point(t).x

    def get_y(self, t: float) -> float:
        return self.get_point(t).y

    def length(self, t: float) -> float:
        return self.part.arc_length() * self._get_t(t)

    def get_unit_tangent_vector(self, t: float) -> Vector2:
        return self.part.unit_tangent_vector(self._get_t(t))

    def project(self, point: Vector2):
        if type(self.part) == Arc:
            part = typing.cast(Arc, self.part)
            return self._inv_t(part.project(point))
        else:
            line = typing.cast(LineSegment, self.part)
            a_sc = (point - line.intersect) * line.slope
            if 0 <= a_sc <= line.max_t:
                return self._inv_t(a_sc / line.max_t)
            raise ValueError

    def __init__(self, part: Union[Arc, LineSegment], min_t: float, max_t: float):
        self.part = part
        self.t_begin = min_t
        self.t_end = max_t


class ArcSpline(Spline):
    def __init__(self, waypoints: List[Pose]):
        super().__init__(waypoints)

    def _calc_half_biarc(self, pm: Vector2, wp: Vector2, t: Vector2, d: float, second_arc: bool):
        n1 = Vector2(-t.y, t.x)
        colinear = (n1 * (pm - wp) * 2)
        if abs(colinear) > 1e-4:
            # Arc
            s1 = ((pm - wp) * (pm - wp) / colinear)
            c1 = wp + n1 * s1
            r1 = abs(s1)
            if r1 == 0:
                theta1 = 0
            else:
                op1 = (wp - c1) / r1
                om1 = (pm - c1) / r1
                cs = op1.cross(om1)
                if d > 0:
                    theta1 = math.acos(op1 * om1) * (1 if cs > 0 else -1)
                else:
                    theta1 = (-2 * math.pi + math.acos(op1 * om1)) * (1 if cs > 0 else -1)
            if not second_arc:
                start_angle1 = (wp - c1).angle()
            else:
                start_angle1 = (pm - c1).angle()
                theta1 *= -1
            return Arc(c1, r1, start_angle1, start_angle1 + theta1)
        else:
            # Line
            if second_arc:
                return LineSegment(pm, wp)
            return LineSegment(wp, pm)

    def _biarc_fit(self, p1: Pose, p2: Pose):
        t1 = p1.unit_tangent_vector()
        t2 = p2.unit_tangent_vector()
        v = p2 - p1

        # Connection point calculation
        denom = 2 * (1 - t1 * t2)
        if denom == 0:
            if v * t1 == 0:
                pm = p1 + v / 2
                c1 = p1 + v / 4
                c2 = p1 + v * (3 / 4)
                r1 = abs(v) / 4
                r2 = r1
                theta1 = math.pi * (1 if v.cross(t2) < 0 else -1)
                theta2 = math.pi * (1 if v.cross(t2) > 0 else -1)
                start1 = (p1 - c1).angle()
                start2 = (p2 - c2).angle()

                return Arc(c1, r1, start1, start1 + theta1), Arc(c2, r2, start2, start2 + theta2)
            else:
                d2 = v * v / (4 * v * t2)
        else:
            t = t1 + t2
            d2 = (-(v * t) + ((v * t) ** 2 + 2 * (1 - t1 * t2) * (v * v)) ** 0.5) / denom
        pm: Vector2 = (p1 + p2 + d2 * (t1 - t2)) / 2

        ret1 = self._calc_half_biarc(pm, p1, t1, d2, False)
        ret2 = self._calc_half_biarc(pm, p2, t2, d2, True)
        return ret1, ret2

    def reticulate(self):
        all_parts = []
        lengths = []
        for i in range(len(self.waypoints) - 1):
            p1: Pose = self.waypoints[i]
            p2: Pose = self.waypoints[i + 1]
            part1, part2 = self._biarc_fit(p1, p2)
            all_parts += [part1, part2]
            lengths += [part1.arc_length(), part2.arc_length()]
        total_len = sum(lengths)
        t_accum = 0
        for i in range(len(all_parts)):
            part_len = lengths[i]
            part = all_parts[i]
            t = part_len / total_len
            self.parts += [ArcPart(part, t_accum, t_accum + t)]
            t_accum += t
        self.length = total_len

    def get_part(self, t: float):
        return super().get_part(t)

    def get_point(self, t: float):
        return super().get_point(t)

    def get_unit_tangent_vector(self, t: float):
        return super().get_unit_tangent_vector(t)

    def arc_length(self, t: float) -> float:
        return super().arc_length(t)

    def get_closest_t_to(self, point: Vector2) -> Tuple[float, float]:
        min_dist = 1e100
        min_t = 0
        for _part in self.parts:
            try:
                t = _part.project(point)
            except ValueError:
                continue
            dist = point.distance(self.get_point(t))
            if dist < min_dist:
                min_t = t
                min_dist = dist
        if min_dist == 1e100:
            print("!! Warning, dropped out of closest-point !!")
            a_sc = (point - self.get_point(1)) * self.get_unit_tangent_vector(1)
            min_t = 1 + a_sc
            return min_t, point.distance(self.get_point(min_t))
        return min_t, min_dist
