import math
import matplotlib.pyplot as plot

import mathlib
from pose import Pose

if __name__ == '__main__':
    waypoints = [Pose(0, 0, 3 * math.pi / 4), Pose(10, 10, -math.pi/4), Pose(15, 5, 0), Pose(20, -10, -math.pi/4)]
    spline = mathlib.ComboSpline(waypoints)
    for wp in waypoints:
        plot.plot(wp.x, wp.y, 'bo')

    xs = []
    ys = []
    for t in range(1000):
        pt = spline.get_point(t/1000)
        xs += [pt.x]
        ys += [pt.y]
    plot.plot(xs, ys)

    plot.show()