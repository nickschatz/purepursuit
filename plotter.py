import math

from pursuit import Waypoint, Pose, curvature


def radius_ratio(R, D):
    return (R - D/2)/(R + D/2)


if __name__ == '__main__':
    path = [Waypoint(0, 0), Waypoint(12, 12)]
    pose = Pose(0, 0, math.pi/4)
    width = 1
    speed = 1
    dt = 1/100
    time = 0

    while pose.distance(path[-1]) > 1e-2:
        time += dt
        if time >= 20:
            break

        curve = curvature(pose, path, 1)
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

        print("{}, {}, {}".format(time, pose.x, pose.y))

