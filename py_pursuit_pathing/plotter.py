import math
import matplotlib.pylab as plot
import time

from py_pursuit_pathing.pursuit import Pose, PurePursuitController, InterpolationStrategy


def radius_ratio(R, D):
    return (R - D/2)/(R + D/2)


if __name__ == '__main__':
    travel_x = []
    travel_y = []
    target_x = []
    target_y = []
    targetp_x = []
    targetp_y = []
    distance = []
    lookahead_percent = []
    times = []
    width = 29.25 / 12
    max_speed = 13
    acc = 10
    hopper_x = -2.47 + width / 2 - width
    hopper_y = 6 + 6.5/12 + width / 2
    path = [Pose(x=1.5, y=-10.0, heading=0.0), Pose(x=16.5, y=-10.0, heading=0.0), Pose(x=19.5, y=0.0, heading=1.5707963267948966), Pose(x=19.5, y=4.0, heading=1.5707963267948966), Pose(x=24.0, y=7.0, heading=-0.2617993877991494)]

    pose = Pose(1.5, -10, 0 * math.pi/4)
    speed = 0

    lookahead = 2
    dt = 1/1000
    controller_ms = 1000/100
    current_time = 0
    loop_ct = 0
    spline = None

    curvature_times = []

    pursuit = PurePursuitController(path, lookahead, interpol_strat=InterpolationStrategy.BIARC,
                                    cruise_speed=max_speed, acc=acc)
    pursuit.init()
    print("Done with spline")

    curve = 0
    start = time.perf_counter()
    while not pursuit.is_at_end(pose):

        if current_time >= 10:
            break
        #try:
        if loop_ct % controller_ms == 0:
            curvature_perf = time.perf_counter()
            curve, cte, speed, goal_pt = pursuit.curvature(pose)
            curvature_times.append((time.perf_counter() - curvature_perf) * 1000)
        if speed < 0.1:
            speed = 0.1
        if curve == 0:
            left_speed = right_speed = speed
        else:
            radius = 1 / -curve
            if radius > 0:
                left_speed = speed
                right_speed = speed * radius_ratio(radius, width)
            else:
                right_speed = speed
                left_speed = speed * radius_ratio(-radius, width)

        left_dist = left_speed * dt
        right_dist = right_speed * dt
        vel = (left_speed + right_speed) / 2
        angular_rate = (right_speed - left_speed) / width

        pose.x += dt * speed * math.cos(pose.heading)
        pose.y += dt * speed * math.sin(pose.heading)
        pose.heading += dt * angular_rate
        # print("{}\t{}\t{}\t{}\t{}\t{}".format(time, pose.x, pose.y, target.x, target.y, pose.distance(path[-1])))

        travel_x += [pose.x]
        travel_y += [pose.y]
        distance += [pose.distance(path[-1])]
        times += [current_time]
        current_time += dt
        loop_ct += 1
    elapsed_realtime = time.perf_counter() - start
    print(f"Average curvature cycle time: {sum(curvature_times)/len(curvature_times)} ms")
    print(f"Total curvature cycle time: {sum(curvature_times)} ms")
    print("Simulated {} seconds in {} real seconds".format(current_time, elapsed_realtime))
    plot.figure(2)

    spline = pursuit.path.spline
    xs = []
    ys = []
    for t in range(1000):
        pt = spline.get_point(t / 1000)
        xs += [pt.x]
        ys += [pt.y]
    do_charts = False
    if do_charts:
        plot.plot(xs, ys)

        plot.plot(travel_x, travel_y)
        plot.plot(target_x, target_y)

        for wp in path:
            plot.plot(wp.x, wp.y, 'bo')

        plot.show()


