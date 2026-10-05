from __future__ import annotations

import math
from typing import List

from src.models import CarPose, Cone, Path2D


class PathPlanning:

    def __init__(self, car_pose: CarPose, cones: List[Cone]):
        self.car_pose = car_pose
        self.cones = cones
        self.default_half_width = 1.5
        self.max_gate_width = 6.0  # blue/yellow cones farther apart than this aren't a gate

    def generatePath(self) -> Path2D:
        # 1. Filter cones by color
        yellow_cones = [c for c in self.cones if c.color == 0]
        blue_cones = [c for c in self.cones if c.color == 1]

        # 2. Sort cones by distance from the car
        def get_distance_to_car(c):
            return math.hypot(c.x - self.car_pose.x, c.y - self.car_pose.y)

        yellow_cones.sort(key=get_distance_to_car)
        blue_cones.sort(key=get_distance_to_car)

        path: Path2D = [(self.car_pose.x, self.car_pose.y)]

        # CASE 1: no cones -> go straight along the car's heading
        if not yellow_cones and not blue_cones:
            path.append((
                self.car_pose.x + 3.0 * math.cos(self.car_pose.yaw),
                self.car_pose.y + 3.0 * math.sin(self.car_pose.yaw),
            ))
            return path

       
        all_pairs = []
        for b in blue_cones:
            for y in yellow_cones:
                all_pairs.append((math.hypot(b.x - y.x, b.y - y.y), b, y))
        all_pairs.sort(key=lambda p: p[0])

        gates = []
        used = []  # cones that are already part of a gate
        for dist, b, y in all_pairs:
            if dist > self.max_gate_width or b in used or y in used:
                continue
            gates.append((b, y))
            used.append(b)
            used.append(y)

        # nearest gate first
        def calculate_gate_distance(g):
            mid_x = (g[0].x + g[1].x) / 2
            mid_y = (g[0].y + g[1].y) / 2
            midpoint = Cone(mid_x, mid_y, 0)
            return get_distance_to_car(midpoint)

        gates.sort(key=calculate_gate_distance)

        # track half-width = half the width of the nearest gate
        half_width = self.default_half_width
        if gates:
            b, y = gates[0]
            half_width = math.hypot(b.x - y.x, b.y - y.y) / 2.0

        # CASE 2: full gates so add the midpoint of each one
        for b, y in gates:
            path.append(((b.x + y.x) / 2.0, (b.y + y.y) / 2.0))

        # CASE 3:  blue cones only (left boundary so offset to the right)
        leftover_blue = [c for c in blue_cones if c not in used]
        self.add_offset_points(path, blue_cones, leftover_blue, -1, half_width, used)

        # CASE 4: leftover yellow cones (right boundary -> offset to the left)
        leftover_yellow = [c for c in yellow_cones if c not in used]
        self.add_offset_points(path, yellow_cones, leftover_yellow, +1, half_width, used)

        # 4. Put the waypoints in driving order (nearest first) and drop duplicates
        start = path[0]
        waypoints = []
        for p in path[1:]:
            if p not in waypoints:
                waypoints.append(p)
        waypoints.sort(key=lambda p: math.hypot(p[0] - start[0], p[1] - start[1]))
        path = [start] + waypoints

        # 5. If the first waypoint is behind the car, first drive a bit forward
        if len(path) > 1:
            angle_to_first = math.atan2(path[1][1] - path[0][1], path[1][0] - path[0][0])
            diff = angle_to_first - self.car_pose.yaw
            diff = math.atan2(math.sin(diff), math.cos(diff))  # wrap to [-pi, pi]
            if abs(diff) > math.pi / 2:
                path.insert(1, (
                    self.car_pose.x + 1.5 * math.cos(self.car_pose.yaw),
                    self.car_pose.y + 1.5 * math.sin(self.car_pose.yaw),
                ))

        return path

    def add_offset_points(self, path, same_color, leftover, side, half_width, used):
        for cone in leftover:
            i = same_color.index(cone)

            # direction of the boundary line (using a neighbouring cone of the same color)
            if i > 0:
                other = same_color[i - 1]
                heading = math.atan2(cone.y - other.y, cone.x - other.x)
            elif len(same_color) > 1:
                other = same_color[i + 1]
                heading = math.atan2(other.y - cone.y, other.x - cone.x)
            else:
                # only one cone of this color: use direction from the last waypoint
                heading = math.atan2(cone.y - path[-1][1], cone.x - path[-1][0])

            # push 90 degrees sideways from the boundary line
            angle = heading + side * math.pi / 2.0
            dx = half_width * math.cos(angle)
            dy = half_width * math.sin(angle)

            # if the previous cone was part of a gate, offset it too
            if i > 0 and same_color[i - 1] in used:
                path.append((same_color[i - 1].x + dx, same_color[i - 1].y + dy))

            path.append((cone.x + dx, cone.y + dy))
