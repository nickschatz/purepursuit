import math

import matplotlib.pyplot as plot

from py_pursuit_pathing import splines, pursuit
from py_pursuit_pathing.mathlib import Vector2, Arc
from py_pursuit_pathing.pose import Pose

if __name__ == '__main__':
    arc1 = Arc(center=Vector2(20.915119338399677, -3.9151193383996747),
               radius=5.0697868511242135, start_angle=-1.423821136131392,
               end_angle=-0.7853981633974481)
    test = Vector2(x=21.76919515745494, y=-8.808543234239727)
    test_angle = (arc1.center - test).angle() - math.pi
    print(test_angle)
    print(arc1.invert(test_angle))
    xs = []
    ys = []
    beg = arc1.r(0)
    end = arc1.r(1)
    plot.plot(beg.x, beg.y, 'bo')
    plot.plot(end.x, end.y, 'ro')
    for k in range(1000):
        t = k/1000
        pt = arc1.r(t)
        xs += [pt.x]
        ys += [pt.y]
    plot.plot(xs, ys)
    plot.show()


