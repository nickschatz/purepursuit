from typing import List, Tuple
import math


class Waypoint:
    """
    Waypoint for path planning. x,y are in world coordinates
    """

    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y

    def translated(self, pose):
        dx = self.x - pose.x
        dy = self.y - pose.y
        dxp = dx * math.cos(pose.heading) - dy * math.sin(pose.heading)
        dyp = dx * math.sin(pose.heading) + dy * math.cos(pose.heading)
        return Waypoint(dxp, dyp)

    def inv_translate(self, pose):
        dxp = self.x * math.cos(pose.heading) + self.y * math.sin(pose.heading)
        dyp = self.x * -math.sin(pose.heading) + self.y * math.cos(pose.heading)
        return Waypoint(dxp + pose.x, dyp + pose.y)

    def distance(self, point):
        return ((point.x - self.x)**2 + (point.y - self.y)**2)**0.5

    def __repr__(self):
        return "Waypoint({}, {})".format(self.x, self.y)

    def __sub__(self, other):
        assert type(other) == Waypoint
        return Waypoint(self.x - other.x, self.y - other.y)

    def __abs__(self):
        return self.distance(Waypoint(0, 0))


class Pose(Waypoint):
    """
    Robot pose in world coordinates
    """
    def __init__(self, x: float, y: float, heading: float):
        super().__init__(x, y)
        self.heading = heading


def line_circ_intercepts(p1: Waypoint, p2: Waypoint, r: float) -> Tuple[Waypoint, Waypoint]:
    dx = p2.x - p1.x
    dy = p2.y - p1.y
    dr = (dx**2 + dy**2)**0.5
    D = p1.x * p2.y - p2.x * p1.y
    discriminant = r**2 * dr**2 - D**2
    if discriminant < 0:
        # print ("Circle and line do not intersect {} {} ".format(p1, p2))
        raise ValueError("Circle and line do not intersect {} {}".format(p1, p2))
    x_pm = (-1 if dy < 0 else 1) * dx * discriminant ** 0.5
    y_pm = abs(dy) * discriminant ** 0.5
    x1, x2 = (D * dy + x_pm) / dr**2, (D * dy - x_pm) / dr**2
    y1, y2 = (-D * dx + y_pm) / dr**2, (-D * dx - y_pm) / dr**2
    return Waypoint(x1, y1), Waypoint(x2, y2)


def curvature(pose: Pose, path: List[Waypoint], lookahead: float) -> Tuple[float, Waypoint]:
    # Get lookahead point
    dist_sorted_path = sorted(path, key=lambda x: x.distance(pose))
    closest_point = dist_sorted_path[0]
    next_closest = dist_sorted_path[1]
    idx = path.index(closest_point)
    if idx == len(path) - 1:
        next_point = path[idx - 1]
    else:
        next_point = path[idx + 1]
    next_point = next_point.translated(pose)
    closest_point = closest_point.translated(pose)
    try:
        p1, p2 = line_circ_intercepts(closest_point, next_point, lookahead)
    except ValueError:
        next_point = path[idx - 1].translated(pose)
        p1, p2 = line_circ_intercepts(closest_point, next_point, lookahead)
    # Choose closer to goal
    if p1.distance(next_point) < p2.distance(next_point):
        goal = p1
    else:
        goal = p2
    curv = 2 * goal.y / lookahead**2
    print("{} {} {} {}".format(curv, closest_point.inv_translate(pose), next_point.inv_translate(pose),
                               goal.inv_translate(pose)))

    return curv, goal.inv_translate(pose)
