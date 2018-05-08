from typing import List, Tuple

from scipy import optimize
import numpy as np

from py_pursuit_pathing import mathlib
from py_pursuit_pathing.mathlib import Vector2
from py_pursuit_pathing.motion_profile import MotionProfile
from py_pursuit_pathing.splines import ComboSpline, CubicSpline, LinearSpline, ArcSpline, QuinticSpline

from py_pursuit_pathing.pose import Pose


def flip_waypoints_y(path: List[Pose]):
    news = []
    for wp in path:
        news.append(Pose(wp.x, -wp.y, -wp.heading))
    return news


class Path:
    def __init__(self):
        pass

    def calc_goal(self, pose: Pose,
                  lookahead_radius: float,
                  t_robot: float) -> Tuple[Vector2, float]:
        raise NotImplementedError

    def get_robot_path_position(self, pose: Pose) -> Tuple[float, float]:
        raise NotImplementedError


class SplinePath(Path):
    def __init__(self, waypoints: List[Pose], interpolation_strategy: int):
        super().__init__()
        print(f"Reticulating {interpolation_strategy} of length {len(waypoints)}")
        self.path = waypoints[:]
        if interpolation_strategy == InterpolationStrategy.COMBO4_5:
            self.spline = ComboSpline(self.path)
        elif interpolation_strategy == InterpolationStrategy.CUBIC:
            self.spline = CubicSpline(self.path)
        elif interpolation_strategy == InterpolationStrategy.QUINTIC:
            self.spline = QuinticSpline(self.path)
        elif interpolation_strategy == InterpolationStrategy.LINEAR:
            self.spline = LinearSpline(self.path)
        elif interpolation_strategy == InterpolationStrategy.BIARC:
            self.spline = ArcSpline(self.path)
        else:
            raise ValueError(f"Invalid interpolation strategy {interpolation_strategy}")

    def get_robot_path_position(self, pose: Pose) -> Tuple[float, float]:
        return self.spline.get_closest_t_to(pose)

    def calc_goal(self, pose: Pose,
                  lookahead_radius: float, t_robot: float):

        # Find intersection
        t_guess = t_robot + lookahead_radius / self.spline.length
        pt = self.spline.get_point(t_guess)
        # line = mathlib.LineSegment(pose, pt)
        # pt = line.r(lookahead_radius / line.max_t)

        if False:
            min_dist_sq = 1e10
            # This can't be easily sped up, at least not without sacrificing a lot of resolution on the
            # lookahead point calculation
            t_granularity = int(self.spline.length * 5)
            t_min = t_robot
            for t_ in range(int(t_robot * t_granularity), t_granularity + 15):
                t = t_ / t_granularity
                pt = self.spline.get_point(t)
                dist_sq = abs(pt.sq_dist(pose) - lookahead_radius**2)
                if dist_sq < min_dist_sq:
                    min_dist_sq = dist_sq
                    t_min = t
            pt = self.spline.get_point(t_min)

        dist = pt.distance(pose)
        return pt, dist


class InterpolationStrategy:
    LINEAR = 0
    CUBIC = 1
    QUINTIC = 2
    COMBO4_5 = 3
    BIARC = 4


class PurePursuitController:
    def __init__(self, waypoints: List[Pose],
                 lookahead_base: float,
                 cruise_speed: float,
                 acc: float,
                 interpol_strat: int = InterpolationStrategy.BIARC):
        self.lookahead_base = lookahead_base
        self.path = SplinePath(waypoints, interpol_strat)
        self.waypoints = waypoints
        self.unpassed_waypoints = waypoints[:]
        self.end_point = waypoints[-1]
        self.acc = acc
        self.cruise_speed = cruise_speed
        self.speed_profile = MotionProfile(start=0, end=self.get_path_length(),
                                           cruise_speed=cruise_speed, acc=acc)
        self.cte = 0
        self.current_lookahead = 0

    def init(self):
        """
        Resets passed waypoints
        :return:
        """
        self.unpassed_waypoints = self.waypoints[:]
        self.cte = 0

    def get_path_length(self):
        return self.path.spline.length

    def lookahead(self, speed: float, err: float) -> float:
        """
        Calculate the lookahead distance based on the robot speed
        :param speed: Robot speed, from 0.0 to 1.0 as a percent of the max speed
        :return: Radius of the lookahead circle
        """
        min_lookahead = 1.2
        return min_lookahead + \
               (speed / self.cruise_speed) * (self.lookahead_base - min_lookahead)

    def curvature(self, pose: Pose) -> Tuple[float, float, float, Vector2]:
        """
        Calculate the curvature of the arc needed to continue following the path
        curvature is 1/(radius of turn)
        :param pose: The robot's pose
        :param speed: The speed of the robot, from 0.0 to 1.0 as a percent of max speed
        :return: The curvature of the path, the cross track error, and the speed at which to drive at (feet/sec)
        """

        t_robot, self.cte = self.path.get_robot_path_position(pose)
        mp_idx = round(t_robot * len(self.speed_profile))
        if mp_idx >= len(self.speed_profile):
            mp_idx = len(self.speed_profile)-1
        if mp_idx < 0:
            mp_idx = 0
        mp_point = self.speed_profile[mp_idx]
        speed = mp_point.velocity

        lookahead_radius = self.lookahead(speed, self.cte)
        self.current_lookahead = lookahead_radius
        goal, dist = self.path.calc_goal(pose, lookahead_radius, t_robot)

        # We're probably only going to pass one waypoint per loop (or have multiple chances to "pass" a waypoint)
        # We need to keep track of the waypoints so we know when we can go to the end
        for point in self.unpassed_waypoints:
            if pose.distance(point) < self.lookahead_base:
                self.unpassed_waypoints.remove(point)
                break

        goal_rs = goal.translated(pose)

        goaly = goal_rs.y
        goal_rs_dist = goal_rs.distance(Vector2(0,0))
        # speed -= abs(goaly)
        try:
            curv = 2 * goaly / (goal_rs_dist ** 2)
        except ZeroDivisionError:
            curv = 0
        return curv, dist, speed, goal

    def is_approaching_end(self, pose):
        return len(self.unpassed_waypoints) == 0

    def is_at_end(self, pose, dist_margin=3/12):
        """
        See if the robot has completed its path
        :param pose: The robot pose
        :return: True if we have gone around the path and are near the end
        """
        t, err = self.path.get_robot_path_position(pose)
        return abs((t * self.get_path_length()) - self.get_path_length()) < dist_margin

    def get_endcte(self, pose):
        return self.end_point.translated(pose).y

    def get_cte(self):
        return self.cte
