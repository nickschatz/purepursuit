import math

from mathlib import Vector2


class Pose(Vector2):
    """
    Robot pose in world coordinates
    """
    def __init__(self, x: float, y: float, heading: float):
        super().__init__(x, y)
        self.heading = heading

    def translated(self, pose):
        vec = super().translated(pose)
        dir_vec = Vector2(math.cos(self.heading), math.sin(self.heading)).translated(pose)
        theta = math.atan2(dir_vec.y, dir_vec.x)

        return Pose(vec.x, vec.y, self.heading - pose.heading)

    def __repr__(self):
        return "Pose(x={}, y={}, heading={})".format(self.x, self.y, self.heading)