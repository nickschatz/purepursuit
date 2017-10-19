import math
import matplotlib.pylab as plot
from pursuit import Vector2, Pose, curvature


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
    times = []
    path = [Vector2(0, 0), Vector2(2.1, 0), Vector2(5.1, 3)]
    pose = Pose(0, 0, 0 * math.pi/4)
    width = 1
    speed = 30
    lookahead = 2
    dt = 1/100
    time = 0
    lines = []

    while pose.distance(path[-1]) > 1e-2:
        time += dt
        if time >= 30:
            break
        try:
            curve, target, lines = curvature(pose, path, lookahead)
        except ValueError:
            print("Break")
            break
        if curve == 0:
            left_speed = right_speed = speed
        else:
            radius = 1 / curve
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

        target_x += [target.x]
        target_y += [target.y]
        target = target.translated(pose)
        targetp_x += [target.x + pose.x]
        targetp_y += [target.y + pose.y]
        travel_x += [pose.x]
        travel_y += [pose.y]
        distance += [pose.distance(path[-1])]
        times += [time]
    plot.figure(1)
    plot.plot(travel_x, travel_y)
    plot.plot(targetp_x, targetp_y)
    for line in lines:
        line.plot(plot)
    for wp in path:
        plot.plot(wp.x, wp.y, 'bo')
    for k in range(len(travel_x)):
        plot.plot([travel_x[k], target_x[k]], [travel_y[k], target_y[k]])
    plot.figure(2)
    plot.plot(travel_x, travel_y)
    plot.plot(targetp_x, targetp_y)
    plot.plot(target_x, target_y)
    for line in lines:
        line.plot(plot)
    for wp in path:
        plot.plot(wp.x, wp.y, 'bo')
    plot.figure(3)
    # plot.plot(distance)
    #plot.plot(travel_x)
    plot.plot(travel_y)
    #plot.figure(2)
    #plot.plot(distance)
    plot.plot(target_y)
    plot.show()


