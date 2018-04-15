import math
import matplotlib.pyplot as plot

import mathlib
import splines
from pose import Pose

if __name__ == '__main__':
    waypoints = [Pose(0, 0, 0 * math.pi / 4), Pose(10, 10, -math.pi/4), Pose(15, 5, 0), Pose(20, -10, -math.pi/4)]
    spline = splines.ComboSpline(waypoints)
    spline2 = splines.CubicSpline(waypoints)
    print(spline2.parts[1].curve)
    reference_frame = Pose(0, 0, 0)
    for wp in map(lambda x: x.translated(reference_frame), waypoints):
        plot.plot(wp.x, wp.y, 'bo')

    xs = []
    ys = []

    xs2 = []
    ys2 = []
    for t in range(1000):
        pt = spline.get_point(t/1000).translated(reference_frame)
        xs += [pt.x]
        ys += [pt.y]

        pt2 = spline2.get_point(t / 1000).translated(reference_frame)
        xs2 += [pt2.x]
        ys2 += [pt2.y]

    plot.plot(xs, ys)
    plot.plot(xs2, ys2)

    plot.show()