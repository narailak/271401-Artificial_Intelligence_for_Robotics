"""Strict, map-free data contracts shared by setup, simulation and ROS."""
from dataclasses import asdict, dataclass, field
import hashlib
import json
import math
from pathlib import Path


def finite(*values):
    return all(type(x) in (int, float) and math.isfinite(x) for x in values)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def digest(data):
    return hashlib.sha256(json.dumps(data, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw: float = 0.0

    def __post_init__(self):
        if not finite(self.x, self.y, self.yaw):
            raise ValueError("Pose must contain finite x, y and yaw")


@dataclass(frozen=True)
class Arena:
    width: float
    height: float
    origin_x: float = 0.0
    origin_y: float = 0.0
    yaw: float = 0.0

    def __post_init__(self):
        if not finite(*asdict(self).values()) or not (.3 <= self.width <= 10 and .3 <= self.height <= 10):
            raise ValueError("Arena dimensions must be 0.3..10 metres")

    def local_to_map(self, x, y):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return self.origin_x+c*x-s*y, self.origin_y+s*x+c*y

    def map_to_local(self, x, y):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        dx, dy = x-self.origin_x, y-self.origin_y
        return c*dx+s*dy, -s*dx+c*dy

    def pose_to_map(self, pose):
        return Pose(*self.local_to_map(pose.x, pose.y), wrap(pose.yaw+self.yaw))

    def pose_from_map(self, pose):
        return Pose(*self.map_to_local(pose.x, pose.y), wrap(pose.yaw-self.yaw))

    def contains(self, x, y, radius=0.0):
        return radius <= x <= self.width-radius and radius <= y <= self.height-radius

    def corners(self):
        return [self.local_to_map(x, y) for x, y in ((0, 0), (self.width, 0), (self.width, self.height), (0, self.height))]


@dataclass(frozen=True)
class Policy:
    goal_weight: float = 3.0
    clearance_weight: float = 0.8
    turn_weight: float = 0.25
    revisit_weight: float = 1.0
    preferred_side: int = 1
    wall_distance: float = 0.26
    slowdown_distance: float = 0.35
    hysteresis: float = 0.025
    side_hold: float = 2.0
    progress_window: float = 15.0
    min_progress: float = 0.025
    lookahead: float = 0.6

    def __post_init__(self):
        if (not finite(*asdict(self).values()) or self.preferred_side not in (-1, 1)
            or not all(0 <= getattr(self, n) <= 10 for n in ('goal_weight', 'clearance_weight', 'turn_weight', 'revisit_weight'))
            or not .17 <= self.wall_distance <= .5 or not .15 <= self.slowdown_distance <= 1
            or not 0 <= self.hysteresis <= 1 or not 0 <= self.side_hold <= 10
            or not 3 <= self.progress_window <= 60 or not .005 <= self.min_progress <= .2
            or not .25 <= self.lookahead <= 1.2):
            raise ValueError("Invalid policy parameters")


@dataclass(frozen=True)
class Hardware:
    # Burger's rear body corner is hypot(0.032 + 0.140/2, 0.140/2) from base_footprint.
    radius: float = 0.124
    safety_margin: float = 0.015
    max_speed: float = 0.05
    max_angular_speed: float = 0.5
    acceleration: float = 0.25
    braking: float = 0.4
    angular_acceleration: float = 2.0
    scan_timeout: float = 0.6
    odom_timeout: float = 0.5
    command_timeout: float = 0.35
    control_period: float = 0.05
    goal_tolerance: float = 0.10
    scan_beams: int = 120
    scan_range: float = 3.5
    lidar_min: float = 0.16
    lidar_x: float = -0.032

    def __post_init__(self):
        if (not finite(*asdict(self).values()) or not .05 <= self.radius <= .5
            or not .015 <= self.safety_margin <= .2 or not 0 < self.max_speed <= .22
            or not 0 < self.max_angular_speed <= 2.84 or not .02 <= self.acceleration <= 1
            or not .02 <= self.braking <= 2 or not .1 <= self.angular_acceleration <= 10
            or not .15 <= self.scan_timeout <= 1 or not .1 <= self.odom_timeout <= 1
            or not .1 <= self.command_timeout <= .5 or not .02 <= self.control_period <= .2
            or not .03 <= self.goal_tolerance <= .3 or type(self.scan_beams) is not int
            or not 60 <= self.scan_beams <= 720 or not .5 <= self.scan_range <= 8
            or not .02 <= self.lidar_min <= .3 or not -.3 <= self.lidar_x <= .3):
            raise ValueError("Hardware outside supported ranges")

    @property
    def safe_radius(self):
        return self.radius+self.safety_margin


@dataclass(frozen=True)
class InitialReference:
    mode: str = 'simulation'
    robot_id: str = 'tb3'
    odom_frame: str = 'odom'
    base_frame: str = 'base_footprint'
    odom_pose: Pose | None = None
    stamp: float = 0.0
    captured_at: float = 0.0
    odom_source: str = ''

    def __post_init__(self):
        if (self.mode not in ('simulation', 'localized', 'manual')
            or not all(isinstance(v, str) and v.strip() and len(v) <= 128 for v in (self.robot_id, self.odom_frame, self.base_frame))
            or self.odom_frame == self.base_frame or not finite(self.stamp, self.captured_at)
            or self.stamp < 0 or self.captured_at < 0
            or not isinstance(self.odom_source,str)
            or self.odom_source and (len(self.odom_source)!=48 or any(c not in '0123456789abcdef' for c in self.odom_source))
            or self.odom_pose is not None and not isinstance(self.odom_pose, Pose)):
            raise ValueError("Invalid robot/odometry reference")
        if self.mode == 'localized' and (self.odom_pose is None or self.stamp <= 0 or self.captured_at <= 0):
            raise ValueError("Localized reference requires captured odometry and timestamp")


SUMMARY_KEYS = {'cases', 'successes', 'collisions', 'training_cases', 'training_successes',
    'training_collisions', 'evaluated_candidates', 'training_seconds', 'evaluation_successes',
    'evaluation_cases', 'evaluation_collisions', 'selected_index', 'baseline_successes',
    'baseline_cases', 'baseline_collisions', 'train_seed', 'evaluation_seed', 'mean_time', 'mean_distance'}


@dataclass(frozen=True)
class Knowledge:
    arena: Arena
    start: Pose
    goal: tuple[float, float]
    policy: Policy = field(default_factory=Policy)
    hardware: Hardware = field(default_factory=Hardware)
    reference: InitialReference = field(default_factory=InitialReference)
    training_summary: dict = field(default_factory=dict)

    def __post_init__(self):
        if (not isinstance(self.arena, Arena) or not isinstance(self.start, Pose)
            or not isinstance(self.policy, Policy) or not isinstance(self.hardware, Hardware)
            or not isinstance(self.reference, InitialReference)
            or len(self.goal) != 2 or not finite(*self.goal)
            or not self.arena.contains(self.start.x, self.start.y, self.hardware.safe_radius)
            or not self.arena.contains(*self.goal, self.hardware.safe_radius)):
            raise ValueError("Start and Goal must fit inside the arena with robot clearance")
        if (not isinstance(self.training_summary, dict) or not set(self.training_summary) <= SUMMARY_KEYS
            or not all(finite(v) and v >= 0 for v in self.training_summary.values())):
            raise ValueError("Training summary must contain only approved aggregate numbers")
        object.__setattr__(self, 'goal', tuple(self.goal))
        object.__setattr__(self, 'training_summary', dict(self.training_summary))

    def payload(self):
        return {'schema_version': 1, 'controller': 'KGGN-1',
                'mission': {'arena': asdict(self.arena), 'start': asdict(self.start), 'goal': list(self.goal)},
                'policy': asdict(self.policy), 'hardware': asdict(self.hardware),
                'initial_reference': asdict(self.reference), 'training_summary': dict(self.training_summary)}

    @property
    def fingerprint(self):
        return digest(self.payload())

    def document(self):
        data = self.payload()
        return {**data, 'fingerprint': digest(data)}

    def save(self, path):
        # Round-trip revalidates mutable summary before exporting.
        data = self.document()
        self.from_document(data)
        path = Path(path).expanduser()
        if path.suffix.lower() != '.json':
            raise ValueError("Knowledge filename must end in .json")
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_name(path.name+'.tmp')
        temporary.write_text(json.dumps(data, indent=2, ensure_ascii=False, allow_nan=False), encoding='utf-8')
        temporary.replace(path)
        return path

    @classmethod
    def from_document(cls, data):
        if not isinstance(data, dict) or set(data) != {'schema_version', 'controller', 'mission', 'policy', 'hardware', 'initial_reference', 'training_summary', 'fingerprint'}:
            raise ValueError("Unexpected knowledge fields; maps and paths are not accepted")
        if data['schema_version'] != 1 or data['controller'] != 'KGGN-1':
            raise ValueError("Unsupported knowledge/controller version")
        if digest({k: v for k, v in data.items() if k != 'fingerprint'}) != data['fingerprint']:
            raise ValueError("Knowledge fingerprint mismatch")
        if set(data['mission']) != {'arena', 'start', 'goal'}:
            raise ValueError("Unexpected mission data")
        reference = dict(data['initial_reference'])
        if reference.get('odom_pose') is not None:
            reference['odom_pose'] = Pose(**reference['odom_pose'])
        return cls(Arena(**data['mission']['arena']), Pose(**data['mission']['start']),
                   tuple(data['mission']['goal']), Policy(**data['policy']), Hardware(**data['hardware']),
                   InitialReference(**reference), data['training_summary'])

    @classmethod
    def load(cls, path):
        path = Path(path).expanduser()
        if path.stat().st_size > 64_000:
            raise ValueError("Knowledge file unexpectedly large")
        try:
            return cls.from_document(json.loads(path.read_text(encoding='utf-8')))
        except (KeyError, TypeError, AttributeError) as exc:
            raise ValueError(f"Invalid knowledge structure: {exc}") from exc
