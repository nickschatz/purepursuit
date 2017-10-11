from typing import List, Tuple, Optional, Union
from mathlib import Line, Vector2


class Pose(Vector2):
    """
    Robot pose in world coordinates
    """
    def __init__(self, x: float, y: float, heading: float):
        super().__init__(x, y)
        self.heading = heading


def line_circ_intercepts(p1: Vector2, p2: Vector2, r: float) -> Tuple[Vector2, Vector2]:
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
    return Vector2(x1, y1), Vector2(x2, y2)


def curvature(pose: Pose, path: List[Vector2], lookahead: float) -> Tuple[float, Vector2]:
    # Get lookahead point
    path_lines = []
    for k in range(len(path) - 1):
        path_lines += [Line(path[k], path[k+1])]
    # For each line if the projected distance is less than lookahead, find points on that line that match lookahead distance
    possible_points = []
    for line in path_lines:
        project = line.projected_point(pose)
        dist = project.distance(pose)
        if dist < lookahead:
            t = line.invert(project)
            d = (lookahead**2 - dist**2)**0.5
            possible_points += [line.r(t + d), line.r(t - d)]
    # Find the closest point to the end of the path
    if len(possible_points) == 0:
        raise Exception("Too far away from path")
    goal = sorted(possible_points, key=lambda x: x.distance(path[-1]))[0]
    if len(possible_points) > 2:
        for k in possible_points:
            if k == goal:
                print("* ", end="")
            print(k.distance(path[-1]))
    curv = 2 * goal.translated(pose).y / lookahead ** 2

    return curv, goal
