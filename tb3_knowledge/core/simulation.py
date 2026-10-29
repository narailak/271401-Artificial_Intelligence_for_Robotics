"""Map-owning simulation; Navigator receives only synthetic live sensors."""
from __future__ import annotations

from collections import deque
import math
import random

from .model import Arena, Hardware, Policy, Pose
from .navigation import Navigator, Observation, wrap
from .scan_filter import bridge_short_dropouts


def _approach(current: float, requested: float, step: float) -> float:
    return current + max(-step, min(step, requested - current))


def _integrate(pose: Pose, velocity: float, angular: float, dt: float) -> Pose:
    if abs(angular) < 1e-9:
        dx, dy = velocity * dt, 0.0
    else:
        dx = velocity / angular * math.sin(angular * dt)
        dy = velocity / angular * (1 - math.cos(angular * dt))
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    return Pose(pose.x + c * dx - s * dy, pose.y + s * dx + c * dy, wrap(pose.yaw + angular * dt))


def run_episode(world, arena: Arena, start: Pose, goal: tuple[float, float], policy: Policy,
                hardware: Hardware, seed: int = 0, max_time: float = 120,
                noise: bool = True) -> dict:
    """Run dynamics, scan/command delay and bounded reproducible disturbances.

    Simulated sensing is 6.7 Hz with 50 ms delivery delay; commands have 50 ms
    delay. Disturbed runs add 4 mm range noise, a small constant odometry scale
    bias and angular bias, and bounded command-delivery jitter. This is a model,
    not calibration evidence for physical hardware.
    """
    if not math.isfinite(max_time) or not 0 < max_time <= 600:
        raise ValueError("Simulation time must be between 0 and 600 seconds")
    if not world.free(start.x, start.y, hardware.safe_radius) or not arena.contains(start.x, start.y, hardware.safe_radius):
        raise ValueError("Start footprint is blocked")
    if not world.free(*goal, hardware.safe_radius) or not arena.contains(*goal, hardware.safe_radius):
        raise ValueError("Goal footprint is blocked")
    rng = random.Random(seed)
    dt = hardware.control_period
    navigator = Navigator(arena, goal, policy, hardware)
    truth = estimate = start
    velocity = angular = 0.0
    applied = (0.0, 0.0)
    commands: deque = deque()
    scans: deque = deque()
    last_scan = None
    next_scan = 0.0
    odom_scale = 1 + (rng.uniform(-0.008, 0.008) if noise else 0)
    odom_angular_scale = 1 + (rng.uniform(-0.006, 0.006) if noise else 0)
    yaw_bias = rng.uniform(-0.0008, 0.0008) if noise else 0
    distance, collisions = 0.0, 0
    trace = [(truth.x, truth.y, truth.yaw)]
    estimated_trace = [(estimate.x, estimate.y, estimate.yaw)]
    elapsed = 0.0
    status = "timeout"
    mode_counts: dict[str, int] = {}
    decisions = []
    for step in range(int(math.ceil(max_time / dt)) + 1):
        now = step * dt
        elapsed = now
        if now + 1e-8 >= next_scan:
            mount_x = hardware.lidar_x
            sensor_pose = Pose(truth.x + mount_x * math.cos(truth.yaw), truth.y + mount_x * math.sin(truth.yaw), truth.yaw)
            ranges = tuple(world.scan(sensor_pose, hardware.scan_beams, hardware.scan_range))
            if noise:
                ranges = tuple(max(0.001, min(hardware.scan_range, r + rng.gauss(0, 0.004))) for r in ranges)
            scans.append((now + 0.05, now, estimate, ranges))
            next_scan += max(0.15, dt)
        while scans and scans[0][0] <= now + 1e-8:
            last_scan = scans.popleft()
        request = (0.0, 0.0)
        if last_scan is not None:
            _, stamp, captured_estimate, ranges = last_scan
            # Compensate scan age using odometry, not simulator ground truth.
            filtered,_,spacing=bridge_short_dropouts(ranges,2*math.pi/len(ranges),0.,hardware.scan_range)
            source = Observation(captured_estimate, filtered, stamp=stamp, mount=(hardware.lidar_x, 0, 0))
            endpoints, invalid = source.sensor_points(hardware.scan_range)
            c0, s0 = math.cos(captured_estimate.yaw), math.sin(captured_estimate.yaw)
            c1, s1 = math.cos(estimate.yaw), math.sin(estimate.yaw)
            points = []
            for px, py in endpoints:
                wx = captured_estimate.x + c0 * px - s0 * py - estimate.x
                wy = captured_estimate.y + s0 * px + c0 * py - estimate.y
                points.append((c1 * wx + s1 * wy, -s1 * wx + c1 * wy))
            observation = Observation(estimate, ranges, stamp=stamp,
                mount=(hardware.lidar_x, 0, 0), linear_speed=velocity, angular_speed=angular,
                points=tuple(points), beam_spacing=spacing,
                invalid_angles=tuple(wrap(a + captured_estimate.yaw - estimate.yaw) for a in invalid))
            request = navigator.command(observation, now)
            mode_counts[navigator.mode] = mode_counts.get(navigator.mode, 0) + 1
            if step % max(1, round(0.5 / dt)) == 0:
                decisions.append({'time': round(now, 3), 'mode': navigator.mode,
                    'reason': navigator.reason, 'v': round(request[0], 5), 'w': round(request[1], 5),
                    'scan_age': round(now - stamp, 3), 'goal_error': round(math.hypot(goal[0] - estimate.x, goal[1] - estimate.y), 4)})
        delivery = now + 0.05 + (rng.uniform(0, 0.025) if noise else 0)
        commands.append((delivery, request))
        while commands and commands[0][0] <= now + 1e-8:
            _, applied = commands.popleft()
        acceleration = hardware.acceleration if applied[0] > velocity else hardware.braking
        velocity = _approach(velocity, applied[0], acceleration * dt)
        angular = _approach(angular, applied[1], hardware.angular_acceleration * dt)
        new_truth = _integrate(truth, velocity, angular, dt)
        if not arena.contains(new_truth.x, new_truth.y, hardware.radius) or not world.free(new_truth.x, new_truth.y, hardware.radius):
            collisions = 1
            status = "collision"
            break
        distance += math.hypot(new_truth.x - truth.x, new_truth.y - truth.y)
        truth = new_truth
        estimate = _integrate(estimate, velocity * odom_scale, angular * odom_angular_scale + (yaw_bias if abs(velocity) + abs(angular) > 0.001 else 0), dt)
        if step % max(1, round(0.1 / dt)) == 0:
            trace.append((truth.x, truth.y, truth.yaw))
            estimated_trace.append((estimate.x, estimate.y, estimate.yaw))
        if navigator.reason == "arrived_estimate" and abs(velocity) < 0.001 and abs(angular) < 0.005:
            physical_error = math.hypot(goal[0] - truth.x, goal[1] - truth.y)
            status = "success" if physical_error <= hardware.goal_tolerance else "estimated_arrival_error"
            break
        if navigator.reason in ("recovery_exhausted", "outside_safe_arena"):
            status = navigator.reason
            break
    if trace[-1] != (truth.x, truth.y, truth.yaw):
        trace.append((truth.x, truth.y, truth.yaw))
        estimated_trace.append((estimate.x, estimate.y, estimate.yaw))
    return {'status': status, 'time': round(elapsed, 4), 'distance': round(distance, 5),
        'collisions': collisions, 'goal_error': round(math.hypot(goal[0] - truth.x, goal[1] - truth.y), 5),
        'recoveries': navigator.recoveries, 'trace': trace, 'estimated_trace': estimated_trace,
        'estimated_goal_error': round(math.hypot(goal[0] - estimate.x, goal[1] - estimate.y), 5),
        'physical_goal_tolerance': hardware.goal_tolerance,
        'mode_counts': mode_counts, 'decisions': decisions, 'last_reason': navigator.reason,
        'seed': seed, 'noise': bool(noise)}
