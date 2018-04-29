import math
import matplotlib.pyplot as plot

import mathlib
import splines
from pose import Pose

if __name__ == '__main__':
    waypoints = [Pose(x=1.5, y=-10.0, heading=0.0), Pose(x=18.0, y=-8.0, heading=0.5235987755982988), Pose(x=20.0, y=5.0, heading=1.5707963267948966), Pose(x=20.5, y=6.0, heading=0.7853981633974483), Pose(x=24.0, y=7.5, heading=-0.17453292519943295)]

    spline2 = splines.CubicSpline(waypoints)
    spline1 = splines.QuinticSpline(waypoints)
    reference_frame = Pose(0, 0, 0)
    for wp in map(lambda x: x.translated(reference_frame), waypoints):
        plot.plot(wp.x, wp.y, 'bo')

    xs = []
    ys = []

    xs2 = []
    ys2 = []

    xs3 = []
    ys3 = []
    for t in range(1000):
        pt = spline1.get_point(t / 1000).translated(reference_frame)
        xs += [pt.x]
        ys += [pt.y]

        pt2 = spline2.get_point(t / 1000).translated(reference_frame)
        xs2 += [pt2.x]
        ys2 += [pt2.y]

        #pt3 = spline3.get_point(t / 1000).translated(reference_frame)
        #xs3 += [pt3.x]
        #ys3 += [pt3.y]

    plot.plot(xs, ys)
    plot.plot(xs2, ys2)
    plot.plot(xs3, ys3)

    plot.show()
