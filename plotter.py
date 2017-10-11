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
    distance = []
    times = []
    path = [Vector2(0, 0), Vector2(6, 0), Vector2(6, 20)]
    pose = Pose(0, 0, 0 * math.pi/4)
    width = 1
    speed = 1
    dt = 1/100
    time = 0

    while pose.distance(path[-1]) > 1e-2:
        time += dt
        if time >= 30:
            break
        curve, target = curvature(pose, path, 1)
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
        dist = (left_dist + right_dist) / 2
        pose.x += dist * math.cos(pose.heading)
        pose.y += dist * math.sin(pose.heading)
        pose.heading += (right_dist - left_dist) / width

        # print("{}\t{}\t{}\t{}\t{}\t{}".format(time, pose.x, pose.y, target.x, target.y, pose.distance(path[-1])))
        travel_x += [pose.x]
        travel_y += [pose.y]
        target_x += [target.x]
        target_y += [target.y]
        distance += [pose.distance(path[-1])]
        times += [time]
    plot.figure(1)
    plot.plot(travel_x, travel_y)
    plot.plot(target_x, target_y)
    for wp in path:
        plot.plot(wp.x, wp.y, 'bo')
    for k in range(len(travel_x)):
        pass  # plot.plot([travel_x[k], target_x[k]], [travel_y[k], target_y[k]])
    # plot.plot(distance)
    #plot.plot(travel_x)
    #plot.plot(travel_y)
    # plot.figure(2)
    # plot.plot(target_x)
    # plot.plot(target_y)
    plot.show()


