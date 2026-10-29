"""Self-contained Program 2 demonstration, independent of the surveyed map.

The generated circle is an illustrative obstacle, not a reconstruction of the
survey. Geometry stays inside this simulator and is never passed to Navigator
or written into Knowledge. This module reads no map or ROS topic.
"""
from __future__ import annotations

import math

from .model import Knowledge, Pose
from .simulation import run_episode


class DemoWorld:
    """Analytic rectangle and optional circular obstacle for offline playback."""

    def __init__(self, knowledge: Knowledge):
        self.arena = knowledge.arena
        self.obstacles: tuple[tuple[float, float, float], ...] = ()
        start, goal = knowledge.start, knowledge.goal
        safe = knowledge.hardware.safe_radius
        distance = math.hypot(goal[0] - start.x, goal[1] - start.y)
        radius = min(.18, .08 * min(self.arena.width, self.arena.height))
        candidates = (((start.x + goal[0]) / 2, (start.y + goal[1]) / 2),
                      (self.arena.width / 2, self.arena.height / 2))
        if distance > 2 * (radius + safe + .03):
            for x, y in candidates:
                # Leave at least a full robot-width corridor around the circle.
                if (self.arena.contains(x, y, radius + 2 * safe + .02)
                        and math.hypot(x - start.x, y - start.y) > radius + safe + .03
                        and math.hypot(x - goal[0], y - goal[1]) > radius + safe + .03):
                    self.obstacles = ((x, y, radius),)
                    break

    def free(self, x: float, y: float, radius: float = 0.0) -> bool:
        if not all(math.isfinite(v) for v in (x, y, radius)) or radius < 0:
            return False
        if not (radius < x < self.arena.width - radius
                and radius < y < self.arena.height - radius):
            return False
        return all(math.hypot(x - cx, y - cy) > radius + cr
                   for cx, cy, cr in self.obstacles)

    def segment_free(self, start, end, radius: float = 0.0) -> bool:
        ax, ay = (start.x, start.y) if isinstance(start, Pose) else start[:2]
        bx, by = (end.x, end.y) if isinstance(end, Pose) else end[:2]
        if not self.free(ax, ay, radius) or not self.free(bx, by, radius):
            return False
        dx, dy = bx - ax, by - ay
        length2 = dx * dx + dy * dy
        for x, y, cr in self.obstacles:
            projection = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / length2)) if length2 else 0
            if math.hypot(x - ax - projection * dx, y - ay - projection * dy) <= cr + radius:
                return False
        return True

    def scan(self, pose: Pose, count: int = 120, max_range: float = 3.5) -> tuple[float, ...]:
        if isinstance(count, bool) or not isinstance(count, int) or count < 1:
            raise ValueError("scan count must be a positive integer")
        if not math.isfinite(max_range) or max_range <= 0:
            raise ValueError("max_range must be positive and finite")
        if not self.free(pose.x, pose.y):
            return (0.0,) * count
        result = []
        for index in range(count):
            angle = pose.yaw - math.pi + index * 2 * math.pi / count
            dx, dy = math.cos(angle), math.sin(angle)
            distance = max_range
            if dx > 1e-12:
                distance = min(distance, (self.arena.width - pose.x) / dx)
            elif dx < -1e-12:
                distance = min(distance, -pose.x / dx)
            if dy > 1e-12:
                distance = min(distance, (self.arena.height - pose.y) / dy)
            elif dy < -1e-12:
                distance = min(distance, -pose.y / dy)
            for x, y, radius in self.obstacles:
                ox, oy = pose.x - x, pose.y - y
                b = ox * dx + oy * dy
                discriminant = b * b - (ox * ox + oy * oy - radius * radius)
                if discriminant >= 0:
                    near = -b - math.sqrt(discriminant)
                    if near >= 0:
                        distance = min(distance, near)
            result.append(max(0.0, distance))
        return tuple(result)


def run_demo(knowledge: Knowledge, seed: int = 2026) -> dict:
    """Exercise the real controller in a generated world, with zero ROS access.

    ``demo_obstacles`` exists only to draw this illustrative world. It must not
    be merged into a Knowledge export or described as the surveyed geometry.
    """
    if not isinstance(knowledge, Knowledge):
        raise TypeError("run_demo requires validated Knowledge")
    world = DemoWorld(knowledge)
    result = run_episode(world, knowledge.arena, knowledge.start, knowledge.goal,
                         knowledge.policy, knowledge.hardware, seed=seed)
    result["demo"] = True
    result["demo_description"] = (
        "สนามตัวอย่างสร้างใหม่ มีสิ่งกีดขวางวงกลมจำลอง ไม่ใช่แผนที่ SLAM จริง"
        if world.obstacles else "สนามตัวอย่างสร้างใหม่ ไม่มีสิ่งกีดขวางภายใน ไม่ใช่แผนที่ SLAM จริง")
    result["demo_obstacles"] = [{"x": x, "y": y, "radius": radius}
                                for x, y, radius in world.obstacles]
    return result
