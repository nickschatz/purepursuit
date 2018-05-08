import matplotlib.pyplot as plot

from py_pursuit_pathing import splines, pursuit
from py_pursuit_pathing.mathlib import Vector2
from py_pursuit_pathing.pose import Pose

if __name__ == '__main__':
    waypoints = [Pose(x=1.5, y=-10.0, heading=0.0), Pose(x=17.0, y=-10.0, heading=0.0), Pose(x=20.0, y=0.0, heading=1.5707963267948966), Pose(x=20.0, y=7.5, heading=1.5707963267948966), Pose(x=24.5, y=7.5, heading=-0.7853981633974483)]

    waypoints = pursuit.flip_waypoints_y(waypoints)
    spline2 = splines.CubicSpline(waypoints)
    spline1 = splines.QuinticSpline(waypoints)
    spline3 = splines.ArcSpline(waypoints)
    print(spline3.parts[-1].part)
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

    for pt in [Vector2(10, -8), Vector2(17, -8), Vector2(17.5, 0), Vector2(21.5, 2),
               Vector2(21.5, 3), Vector2(17.5, 4), Vector2(19.5, 6), Vector2(21.5, 5),
               Vector2(17.5, -4), Vector2(22, 7), Vector2(20, 12), Vector2(20.5, 7.5),
               Vector2(20.5, 8), Vector2(x=20.310088074346652, y=-7.754148353382901),
               Vector2(x=21.76919515745494, y=-8.808543234239727)]:
        try:
            t, dist = spline3.get_closest_t_to(pt)
            close = spline3.get_point(t)

            plot.plot([pt.x, close.x], [pt.y, close.y])
        except ValueError:
            plot.plot(pt.x, pt.y, 'rx')
    plot.show()


