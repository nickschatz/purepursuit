import math

from py_pursuit_pathing.mathlib import Vector2


class Pose(Vector2):
    """
    Robot pose in world coordinates
    """
    def __init__(self, x: float, y: float, heading: float):
        super().__init__(x, y)
        self.heading = heading

    def translated(self, pose):
        vec = super().translated(pose)
        return Pose(vec.x, vec.y, self.heading - pose.heading)

    def unit_tangent_vector(self):
        return Vector2(math.cos(self.heading), math.sin(self.heading)).normalized()

    def __repr__(self):
        return "Pose(x={}, y={}, heading={})".format(self.x, self.y, self.heading)

    def __add__(self, other):
        if type(other) == type(self):
            return Pose(self.x + other.x, self.y + other.y, self.heading + other.heading)
        elif type(other) == Vector2:
            return Vector2(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        if type(other) == Pose:
            return Pose(self.x - other.x, self.y - other.y, self.heading - other.heading)
        elif type(other) == Vector2:
            return Vector2(self.x - other.x, self.y - other.y)

    def copy(self):
        return Pose(self.x, self.y, self.heading)

    def __copy__(self):
        return self.copy()