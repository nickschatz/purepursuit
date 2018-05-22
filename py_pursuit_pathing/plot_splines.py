import matplotlib.pyplot as plot

from py_pursuit_pathing import splines, pursuit
from py_pursuit_pathing.mathlib import Vector2
from py_pursuit_pathing.pose import Pose

if __name__ == '__main__':
    waypoints = [Pose(x=1.5, y=-10.0, heading=0.0), Pose(x=20.0, y=0.0, heading=1.5707963267948966), Pose(x=25.0, y=7.0, heading=0.0)]
    # waypoints = pursuit.flip_waypoints_y(waypoints)
    spline1 = splines.QuinticSpline(waypoints)
    spline2 = splines.approximate_spline(spline1, error=1/12)
    reference_frame = Pose(0, 0, 0)
    for wp in map(lambda x: x.translated(reference_frame), waypoints):
        plot.plot(wp.x, wp.y, 'bo')

    xs = []
    ys = []

    xs2 = []
    ys2 = []

    xs3 = []
    ys3 = []
    resolution = 1000
    for t in range(resolution):
        pt = spline1.get_point(t / resolution).translated(reference_frame)
        xs += [pt.x]
        ys += [pt.y]

        pt2 = spline2.get_point(t / resolution).translated(reference_frame)
        xs2 += [pt2.x]
        ys2 += [pt2.y]

        #pt3 = spline3.get_point(t / resolution).translated(reference_frame)
        #xs3 += [pt3.x]
        #ys3 += [pt3.y]

    plot.plot(xs, ys)
    plot.plot(xs2, ys2)
    # plot.plot(xs3, ys3)
    for x,y in map(lambda x: (x.x, x.y), spline2.waypoints):
        plot.plot(x, y, 'rx')

    ts = []
    curv = []
    curv2 = []
    resolution = 1000
    for t_ in range(resolution):
        t = t_ / resolution
        ts += [t]
        curv += [abs(spline1.curvature(t))]
        curv2 += [abs(spline2.curvature(t))]
    plot.figure(2)
    plot.plot(ts, curv)
    plot.plot(ts, curv2)

    plot.show()




