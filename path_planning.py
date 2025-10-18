from __future__ import annotations

from typing import List

from src.models import CarPose, Cone, Path2D


import math


class PathPlanning:
    """Student-implemented path planner.

    You are given the car pose and an array of detected cones, each cone with (x, y, color)
    where color is 0 for yellow (right side) and 1 for blue (left side). The goal is to
    generate a sequence of path points that the car should follow.

    Implement ONLY the generatePath function.
    """

    def __init__(self, car_pose: CarPose, cones: List[Cone], track_width: float = 3.0):
        """Initialize planner.

        track_width: approximate distance between left and right cones (meters).
        """
        self.car_pose = car_pose
        self.cones = cones
        self.track_width = track_width

    def generatePath(self) -> Path2D:
        """Return a list of path points (x, y) in world frame.

        Requirements and notes:
        - Cones: color==0 (yellow) are on the RIGHT of the track; color==1 (blue) are on the LEFT. 
        - You may be given 2, 1, or 0 cones on each side.
        - Use the car pose (x, y, yaw) to seed your path direction if needed.
        - Return a drivable path that stays between left (blue) and right (yellow) cones.
        - The returned path will be visualized by PathTester.

        The path can contain as many points as you like, but it should be between 5-10 meters,
        with a step size <= 0.5. Units are meters.

        Replace the placeholder implementation below with your algorithm.
        """

        blue = []
        yellow = []

        # Separate cones by color
        for c in self.cones:
            if c.color == 1:
                blue.append(c)
            elif c.color == 0:
                yellow.append(c)

        car = self.car_pose
        W = float(self.track_width)  # track width (m)
        lookahead = 3.0  # forward distance (m)
        path = []

        # --- Helper: compute average position of cones ---
        def average(cones):
            if not cones:
                return None
            sum_x = 0.0
            sum_y = 0.0
            for c in cones:
                sum_x += c.x
                sum_y += c.y
            n = len(cones)
            return (sum_x / n, sum_y / n)

        blue_avg = average(blue)
        yellow_avg = average(yellow)

        # --- Case 1: both sides visible ---
        if blue_avg is not None and yellow_avg is not None:
            cx = (blue_avg[0] + yellow_avg[0]) / 2.0
            cy = (blue_avg[1] + yellow_avg[1]) / 2.0

        # --- Case 2: only blue cones visible ---
        elif blue_avg is not None:
            bx, by = blue_avg
            cx = bx - (W / 2.0) * math.sin(car.yaw)
            cy = by + (W / 2.0) * math.cos(car.yaw)

        # --- Case 3: only yellow cones visible ---
        elif yellow_avg is not None:
            yx, yy = yellow_avg
            cx = yx + (W / 2.0) * math.sin(car.yaw)
            cy = yy - (W / 2.0) * math.cos(car.yaw)

        # --- Case 4: no cones visible ---
        else:
            cx = car.x + lookahead * math.cos(car.yaw)
            cy = car.y + lookahead * math.sin(car.yaw)

        # --- Generate raw path points ---
        path.append((car.x, car.y))
        path.append((cx, cy))

        # Extend forward for smoother visual
        forward_x = cx + lookahead * math.cos(car.yaw)
        forward_y = cy + lookahead * math.sin(car.yaw)
        path.append((forward_x, forward_y))

        # --- Simple smoothing ---
        smooth_path = []
        for i in range(len(path)):
            # average this point with its neighbors if possible
            sum_x = 0.0
            sum_y = 0.0
            count = 0
            for j in range(max(0, i - 1), min(len(path), i + 2)):
                sum_x += path[j][0]
                sum_y += path[j][1]
                count += 1
            smooth_path.append((sum_x / count, sum_y / count))

        return smooth_path
