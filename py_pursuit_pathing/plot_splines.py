import matplotlib.pyplot as plot

from py_pursuit_pathing import splines, pursuit
from py_pursuit_pathing.mathlib import Vector2
from py_pursuit_pathing.pose import Pose

if __name__ == '__main__':
    # waypoints = [Pose(x=1.5, y=-10.0, heading=0.0), Pose(x=17.0, y=-10.0, heading=0.0), Pose(x=20.0, y=0.0, heading=1.5707963267948966), Pose(x=20.0, y=7.5, heading=1.5707963267948966), Pose(x=24.5, y=7.5, heading=-0.7853981633974483)]
    waypoints = [Pose(x=1.5, y=-10.0, heading=0.0), Pose(x=15.0, y=-10.0, heading=0.0), Pose(x=19.0, y=-8.0, heading=0.7853981633974483), Pose(x=20.5, y=6.0, heading=1.3962634015954636), Pose(x=24.0, y=7.0, heading=0.0)]
    # waypoints = pursuit.flip_waypoints_y(waypoints)
    spline1 = splines.QuinticSpline(waypoints)
    spline2 = splines.approximate_spline(spline1)
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
    plot.show()


