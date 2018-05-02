import matplotlib.pyplot as plot

from py_pursuit_pathing import splines
from py_pursuit_pathing.pose import Pose

if __name__ == '__main__':
    waypoints = [Pose(x=1.5, y=-10.0, heading=0.0), Pose(x=16.5, y=-10.0, heading=0.0), Pose(x=19.5, y=0.0, heading=1.5707963267948966), Pose(x=19.5, y=4.0, heading=1.5707963267948966), Pose(x=24.0, y=7.0, heading=-0.2617993877991494)]

    spline2 = splines.CubicSpline(waypoints)
    spline1 = splines.QuinticSpline(waypoints)
    spline3 = splines.ArcSpline(waypoints)
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

        pt3 = spline3.get_point(t / resolution).translated(reference_frame)
        xs3 += [pt3.x]
        ys3 += [pt3.y]

    # plot.plot(xs, ys)
    # plot.plot(xs2, ys2)
    plot.plot(xs3, ys3)

    plot.show()
