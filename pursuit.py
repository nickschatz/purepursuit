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

    def distance(self, point):
        return ((point.x - self.x)**2 + (point.y - self.y)**2)**0.5

    def __repr__(self):
        return "Waypoint({}, {})".format(self.x, self.y)


class Pose(Waypoint):
    """
    Robot pose in world coordinates
    """
    def __init__(self, x: float, y: float, heading: float):
        super().__init__(x, y)
        self.heading = heading


def line_circ_intercepts(p1: Waypoint, p2: Waypoint, r: float) -> Tuple[Waypoint, Waypoint]:
    dx = p2.x - p1.x
    dy = p2.x - p1.x
    dr = (dx**2 + dy**2)**0.5
    D = p1.x * p2.y - p2.x * p1.y
    discriminant = r**2 * dr**2 - D**2
    if discriminant < 0:
        return p2, p2  # hack?
    x_pm = (-1 if dy < 0 else 1) * dx * discriminant ** 0.5
    y_pm = abs(dy) * discriminant ** 0.5
    x1, x2 = (D * dy + x_pm) / dr**2, (D * dy - x_pm) / dr**2
    y1, y2 = (-D * dx + y_pm) / dr**2, (-D * dx - y_pm) / dr**2
    return Waypoint(x1, y1), Waypoint(x2, y2)


def curvature(pose: Pose, path: List[Waypoint], lookahead: float) -> float:
    # Get lookahead point
    closest_point = sorted(path, key=lambda x: x.distance(pose))[0]
    idx = path.index(closest_point)
    if idx == len(path) - 1:
        goal = closest_point
    else:
        next_point = path[idx - 1]
        next_point = next_point.translated(pose)
        closest_point = closest_point.translated(pose)
        p1, p2 = line_circ_intercepts(closest_point, next_point, lookahead)
        # Choose closer to goal
        if p1.distance(next_point) < p2.distance(next_point):
            goal = p1
        else:
            goal = p2

    return 2 * goal.x / lookahead**2
