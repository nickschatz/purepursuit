import math

from py_pursuit_pathing.mathlib import Arc, Vector2, LineSegment
from py_pursuit_pathing.pose import Pose


def calc_half_biarc(pm: Vector2, wp: Vector2, t: Vector2, d: float, second_arc: bool):
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


def biarc_fit(p1: Pose, p2: Pose):
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

    ret1 = calc_half_biarc(pm, p1, t1, d2, False)
    ret2 = calc_half_biarc(pm, p2, t2, d2, True)
    return ret1, ret2
