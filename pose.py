from mathlib import Vector2


class Pose(Vector2):
    """
    Robot pose in world coordinates
    """
    def __init__(self, x: float, y: float, heading: float):
        super().__init__(x, y)
        self.heading = heading

    def __repr__(self):
        return "Pose(x={}, y={}, heading={})".format(self.x, self.y, self.heading)