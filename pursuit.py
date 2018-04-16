import math
from typing import List, Tuple, Optional, Union, Callable

import mathlib
from mathlib import LineSegment, Vector2
from splines import ComboSpline, CubicSpline, Spline, LinearSpline

import copy

from pose import Pose


class Path:
    def __init__(self):
        pass

    def calc_goal(self, pose: Pose,
                  lookahead_radius: float) -> Tuple[Vector2, float]:
        pass


class SplinePath(Path):
    def __init__(self, waypoints: List[Pose], interpolation_strategy: int):
        super().__init__()
        self.path = waypoints[:]
        if interpolation_strategy == InterpolationStrategy.COMBO4_5:
            self.spline = ComboSpline(self.path)
        elif interpolation_strategy == InterpolationStrategy.CUBIC:
            self.spline = CubicSpline(self.path)
        elif interpolation_strategy == InterpolationStrategy.LINEAR:
            self.spline = LinearSpline(self.path)
        else:
            raise ValueError(f"Invalid interpolation strategy {interpolation_strategy}")

    def calc_goal(self, pose: Pose,
                  lookahead_radius: float):

        # Find closest t to pose
        t_robot = 0
        min_dist_sq = 1e10
        # TODO better numerical method for finding close point, if necessary
        t_granularity = int(self.spline.length * 5)
        for t in range(t_granularity):
            t = t/t_granularity
            pt = self.spline.get_point(t)
            dist_sq = pt.sq_dist(pose)
            if dist_sq < min_dist_sq:
                min_dist_sq = dist_sq
                t_robot = t

        # Find intersection
        t_guess = t_robot + lookahead_radius / self.spline.length
        pt = self.spline.get_point(t_guess)
        line = mathlib.LineSegment(pose, pt)
        pt = line.r(lookahead_radius)

        dist = pt.distance(pose)

        return pt, dist


class InterpolationStrategy:
    LINEAR = 0
    CUBIC = 1
    QUINTIC = 2
    COMBO4_5 = 3


class PurePursuitController:
    def __init__(self, pose: Pose, waypoints: List[Pose], lookahead_base: float,
                 interpol_strategy: int=InterpolationStrategy.CUBIC):
        self.pose = pose
        self.lookahead_base = lookahead_base
        self.path = SplinePath(waypoints, interpol_strategy)
        self.waypoints = waypoints
        self.unpassed_waypoints = waypoints[:]
        self.end_point = waypoints[-1]

    def lookahead(self, speed: float) -> float:
        """
        Calculate the lookahead distance based on the robot speed
        :param speed: Robot speed, from 0.0 to 1.0 as a percent of the max speed
        :return: Radius of the lookahead circle
        """
        return self.lookahead_base

    def curvature(self, pose: Pose, speed: float) -> Tuple[float, Vector2, Spline]:
        """
        Calculate the curvature of the arc needed to continue following the path
        curvature is 1/(radius of turn)
        :param pose: The robot's pose
        :param speed: The speed of the robot, from 0.0 to 1.0 as a percent of max speed
        :return: The curvature of the path and the cross track error
        """
        lookahead_radius = self.lookahead(speed)

        # We're probably only going to pass one waypoint per loop (or have multiple chances to "pass" a waypoint)
        # We need to keep track of the waypoints so we know when we can go to the end
        for point in self.unpassed_waypoints:
            if pose.distance(point) < lookahead_radius:
                self.unpassed_waypoints.remove(point)
                break

        goal, dist = self.path.calc_goal(pose, lookahead_radius)
        try:
            curv = -2 * goal.translated(pose).y / dist ** 2
        except ZeroDivisionError:
            curv = 0
        return curv, goal, self.path.spline

    def is_approaching_end(self, pose):
        return len(self.unpassed_waypoints) == 0

    def is_at_end(self, pose, dist_margin=3/12):
        """
        See if the robot has completed its path
        :param pose: The robot pose
        :return: True if we have gone around the path and are near the end
        """
        translated_end = self.end_point.translated(pose)
        err = abs(translated_end.x)
        return pose.distance(self.end_point) < self.lookahead_base and translated_end.x < 0
