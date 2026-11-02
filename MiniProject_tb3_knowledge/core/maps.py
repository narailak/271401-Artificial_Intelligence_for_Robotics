"""Read-only SLAM maps and conservative geometry for Program 1 simulation.

Image coordinates use a top-left origin. ``pixel_to_map(0, 0)`` returns the
*center* of the top-left pixel. The YAML origin is the bottom-left outer edge
of the image, with its stated yaw. Occupied and unknown pixels both block
footprint queries and simulated laser rays.

This module belongs to map preparation/simulation. The live navigation
controller must not receive a ``SlamMap`` or a ``MissionWorld``.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from pathlib import Path
from typing import Sequence

import numpy as np
import yaml

from .model import Arena, Pose


FREE = 0
OCCUPIED = 1
UNKNOWN = 2
_SPACE = b" \t\r\n\v\f"


def _number(value: object, name: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be a finite number")
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{name} must be a finite number") from exc
    if not math.isfinite(result):
        raise ValueError(f"{name} must be a finite number")
    return result


def _pgm(path: Path) -> tuple[np.ndarray, int]:
    """Read P2/P5 without consuming whitespace-valued first binary pixels."""
    data = path.read_bytes()
    cursor = 0

    def token() -> bytes:
        nonlocal cursor
        while cursor < len(data):
            if data[cursor] in _SPACE:
                cursor += 1
            elif data[cursor] == 35:
                newline = data.find(b"\n", cursor)
                cursor = len(data) if newline < 0 else newline + 1
            else:
                break
        begin = cursor
        while cursor < len(data) and data[cursor] not in _SPACE + b"#":
            cursor += 1
        if begin == cursor:
            raise ValueError("PGM header or pixels are incomplete")
        return data[begin:cursor]

    magic = token()
    if magic not in (b"P2", b"P5"):
        raise ValueError("Map image must be a PGM P2 or P5 file")
    try:
        width, height, maximum = (int(token()) for _ in range(3))
    except ValueError as exc:
        raise ValueError("Invalid PGM dimensions or maximum value") from exc
    if width <= 0 or height <= 0 or width * height > 64_000_000:
        raise ValueError("PGM dimensions must be positive and at most 64 million pixels")
    if not 1 <= maximum <= 65535:
        raise ValueError("PGM maximum value must be between 1 and 65535")
    size = width * height
    if magic == b"P2":
        try:
            values = [int(token()) for _ in range(size)]
        except ValueError as exc:
            raise ValueError("PGM pixels are invalid or incomplete") from exc
        # A second raster is not silently accepted as part of this map.
        try:
            token()
        except ValueError:
            pass
        else:
            raise ValueError("Unexpected data after PGM pixels")
        pixels = np.asarray(values, dtype=np.int64)
    else:
        if cursor >= len(data) or data[cursor] not in _SPACE:
            raise ValueError("PGM binary raster needs one whitespace separator")
        # CRLF is one header line ending. Never skip arbitrary whitespace here:
        # samples with values 10, 13, 32 and 35 are legitimate image data.
        cursor += 2 if data[cursor:cursor + 2] == b"\r\n" else 1
        item_size = 1 if maximum < 256 else 2
        stop = cursor + size * item_size
        if stop > len(data):
            raise ValueError("PGM binary pixels are incomplete")
        if data[stop:].strip(_SPACE):
            raise ValueError("Unexpected data after PGM pixels")
        pixels = np.frombuffer(data, dtype=np.uint8 if item_size == 1 else ">u2",
                               count=size, offset=cursor).copy()
    if np.any(pixels < 0) or np.any(pixels > maximum):
        raise ValueError("PGM sample is outside its declared range")
    return pixels.reshape(height, width), maximum


def _point(value: Pose | Sequence[float]) -> tuple[float, float]:
    if hasattr(value, "x") and hasattr(value, "y"):
        return float(value.x), float(value.y)
    return float(value[0]), float(value[1])


@dataclass(frozen=True)
class SlamMap:
    """A trinary map; ``states[row, col]`` uses 0 free, 1 occupied, 2 unknown."""

    states: np.ndarray
    resolution: float
    origin: tuple[float, float, float]
    yaml_path: Path | None = None
    image_path: Path | None = None

    def __post_init__(self) -> None:
        states = np.asarray(self.states)
        if states.ndim != 2 or not states.size or np.any(~np.isin(states, (0, 1, 2))):
            raise ValueError("Map states must be a non-empty 2-D array of 0, 1 or 2")
        resolution = _number(self.resolution, "resolution")
        if resolution <= 0:
            raise ValueError("resolution must be positive")
        if len(self.origin) != 3:
            raise ValueError("origin must contain x, y and yaw")
        origin = tuple(_number(v, "origin") for v in self.origin)
        states = states.astype(np.uint8, copy=True)
        states.setflags(write=False)
        object.__setattr__(self, "states", states)
        object.__setattr__(self, "resolution", resolution)
        object.__setattr__(self, "origin", origin)

    @classmethod
    def load(cls, yaml_path: str | Path, image_path: str | Path | None = None) -> "SlamMap":
        source = Path(yaml_path).expanduser().resolve()
        try:
            metadata = yaml.safe_load(source.read_text(encoding="utf-8"))
            if not isinstance(metadata, dict):
                raise ValueError("Map YAML must contain a mapping")
            if metadata.get("mode", "trinary") != "trinary":
                raise ValueError("Only trinary map mode is supported; scale/raw are not trinary")
            resolution = _number(metadata.get("resolution"), "resolution")
            if resolution <= 0:
                raise ValueError("resolution must be positive")
            raw_origin = metadata.get("origin")
            if not isinstance(raw_origin, (list, tuple)) or len(raw_origin) != 3:
                raise ValueError("origin must contain x, y and yaw")
            origin = tuple(_number(v, "origin") for v in raw_origin)
            free = _number(metadata.get("free_thresh"), "free_thresh")
            occupied = _number(metadata.get("occupied_thresh"), "occupied_thresh")
            if not 0 <= free < occupied <= 1:
                raise ValueError("Thresholds must satisfy 0 <= free_thresh < occupied_thresh <= 1")
            negate = metadata.get("negate", 0)
            if isinstance(negate, bool) or negate not in (0, 1):
                raise ValueError("negate must be 0 or 1")
            if image_path is None:
                image = metadata.get("image")
                if not isinstance(image, str) or not image:
                    raise ValueError("Map YAML needs an image path, or select a PGM separately")
                raster = Path(image).expanduser()
                if not raster.is_absolute():
                    raster = source.parent / raster
            else:
                raster = Path(image_path).expanduser()
            raster = raster.resolve()
            pixels, maximum = _pgm(raster)
            probability = pixels.astype(np.float64) / maximum
            if not negate:
                probability = 1.0 - probability
            states = np.full(pixels.shape, UNKNOWN, dtype=np.uint8)
            states[probability < free] = FREE
            states[probability > occupied] = OCCUPIED
            return cls(states, resolution, origin, source, raster)
        except FileNotFoundError:
            # The setup UI offers a separate PGM picker when a saved YAML's
            # relative image has moved. Keep this distinct from malformed maps.
            raise
        except (OSError, yaml.YAMLError) as exc:
            raise ValueError(f"Cannot load SLAM map: {exc}") from exc

    @property
    def width(self) -> int:
        return int(self.states.shape[1])

    @property
    def height(self) -> int:
        return int(self.states.shape[0])

    def _to_image_axes(self, x: float, y: float) -> tuple[float, float]:
        dx, dy = x - self.origin[0], y - self.origin[1]
        c, s = math.cos(self.origin[2]), math.sin(self.origin[2])
        return c * dx + s * dy, -s * dx + c * dy

    def _from_image_axes(self, x: float, y: float) -> tuple[float, float]:
        c, s = math.cos(self.origin[2]), math.sin(self.origin[2])
        return self.origin[0] + c * x - s * y, self.origin[1] + s * x + c * y

    def map_to_pixel(self, x: float, y: float) -> tuple[float, float]:
        """Continuous center-based image coordinates; values may be outside image."""
        x, y = self._to_image_axes(x, y)
        return x / self.resolution - 0.5, self.height - 0.5 - y / self.resolution

    def pixel_to_map(self, u: float, v: float) -> tuple[float, float]:
        """Map coordinate of a center-based image point (u column, v row)."""
        return self._from_image_axes((u + 0.5) * self.resolution,
                                     (self.height - v - 0.5) * self.resolution)

    def known_arena(self) -> Arena:
        """Non-unknown bounding box, aligned with image axes, not detected walls.

        This is a preview/default selection. The user must confirm the physical
        arena rectangle, especially when walls are tilted relative to the image.
        """
        rows, cols = np.nonzero(self.states != UNKNOWN)
        if not len(rows):
            raise ValueError("Map contains no known cells; choose another map")
        left, right = int(cols.min()), int(cols.max())
        top, bottom = int(rows.min()), int(rows.max())
        x, y = self._from_image_axes(left * self.resolution,
                                    (self.height - bottom - 1) * self.resolution)
        return Arena(width=(right - left + 1) * self.resolution,
                     height=(bottom - top + 1) * self.resolution,
                     origin_x=x, origin_y=y, yaw=self.origin[2])

    def detect_arena(self):
        """Find a rectangle supported by outer walls and exterior unknown."""
        from .boundary import detect_arena
        return detect_arena(self)

    def _local_free(self, x: float, y: float, radius: float) -> bool:
        if not all(math.isfinite(v) for v in (x, y, radius)) or radius < 0:
            return False
        r = self.resolution
        if x - radius <= 0 or y - radius <= 0 or x + radius >= self.width * r or y + radius >= self.height * r:
            return False
        left = max(0, math.floor((x - radius) / r - 1e-10))
        right = min(self.width - 1, math.floor((x + radius) / r + 1e-10))
        bottom = max(0, math.floor((y - radius) / r - 1e-10))
        top = min(self.height - 1, math.floor((y + radius) / r + 1e-10))
        rows, cols = np.nonzero(self.states[self.height - 1 - top:self.height - bottom,
                                           left:right + 1] != FREE)
        if not len(rows):
            return True
        bx = (cols + left) * r
        by = (top - rows) * r
        dx = np.maximum(np.maximum(bx - x, x - bx - r), 0)
        dy = np.maximum(np.maximum(by - y, y - by - r), 0)
        return not bool(np.any(dx * dx + dy * dy <= radius * radius + 1e-14))

    def free(self, x: float, y: float, radius: float = 0.0) -> bool:
        """True only when the closed circular footprint avoids every blocked cell."""
        return self._local_free(*self._to_image_axes(x, y), radius)

    def segment_free(self, a: Pose | Sequence[float], b: Pose | Sequence[float],
                     radius: float = 0.0) -> bool:
        """Exact segment/cell distance test for a circular swept footprint."""
        ax, ay = self._to_image_axes(*_point(a))
        bx, by = self._to_image_axes(*_point(b))
        if not self._local_free(ax, ay, radius) or not self._local_free(bx, by, radius):
            return False
        r = self.resolution
        left = max(0, math.floor((min(ax, bx) - radius) / r - 1e-10))
        right = min(self.width - 1, math.floor((max(ax, bx) + radius) / r + 1e-10))
        bottom = max(0, math.floor((min(ay, by) - radius) / r - 1e-10))
        top = min(self.height - 1, math.floor((max(ay, by) + radius) / r + 1e-10))
        rows, cols = np.nonzero(self.states[self.height - 1 - top:self.height - bottom,
                                           left:right + 1] != FREE)
        if not len(rows):
            return True
        x0, y0 = (cols + left) * r, (top - rows) * r
        x1, y1 = x0 + r, y0 + r
        dx, dy = bx - ax, by - ay
        enter, leave = np.zeros(len(rows)), np.ones(len(rows))
        for start, delta, low, high in ((ax, dx, x0, x1), (ay, dy, y0, y1)):
            if abs(delta) < 1e-15:
                leave[(start < low) | (start > high)] = -1
            else:
                t0, t1 = (low - start) / delta, (high - start) / delta
                enter = np.maximum(enter, np.minimum(t0, t1))
                leave = np.minimum(leave, np.maximum(t0, t1))
        if np.any(enter <= leave):
            return False
        # With no intersection, the shortest distance is attained at an end of
        # either the segment or a rectangle edge (one of its four corners).
        distances = []
        for x, y in ((ax, ay), (bx, by)):
            ex = np.maximum(np.maximum(x0 - x, x - x1), 0)
            ey = np.maximum(np.maximum(y0 - y, y - y1), 0)
            distances.append(ex * ex + ey * ey)
        length2 = dx * dx + dy * dy
        if length2:
            for x, y in ((x0, y0), (x0, y1), (x1, y0), (x1, y1)):
                t = np.clip(((x - ax) * dx + (y - ay) * dy) / length2, 0, 1)
                distances.append((x - ax - t * dx) ** 2 + (y - ay - t * dy) ** 2)
        return not bool(np.any(np.minimum.reduce(distances) <= radius * radius + 1e-14))

    def scan(self, pose: Pose, count: int = 120, max_range: float = 3.5) -> tuple[float, ...]:
        """Uniform full-circle laser, angle_i = -pi + 2*pi*i/count in base frame.

        Returns finite ranges. Unknown pixels and map boundaries are opaque.
        Rays use exact grid traversal instead of stepping over thin obstacles.
        """
        if isinstance(count, bool) or not isinstance(count, int) or not 1 <= count <= 4096:
            raise ValueError("scan count must be an integer between 1 and 4096")
        maximum = _number(max_range, "max_range")
        if maximum <= 0:
            raise ValueError("max_range must be positive")
        if not all(math.isfinite(v) for v in (pose.x, pose.y, pose.yaw)):
            raise ValueError("scan pose must be finite")
        x, y = self._to_image_axes(pose.x, pose.y)
        if not self._local_free(x, y, 0):
            return (0.0,) * count
        r = self.resolution
        initial_x, initial_y = math.floor(x / r), math.floor(y / r)

        def blocked(ix: int, iy: int) -> bool:
            return (ix < 0 or ix >= self.width or iy < 0 or iy >= self.height
                    or self.states[self.height - 1 - iy, ix] != FREE)

        def ray(angle: float) -> float:
            dx, dy = math.cos(angle), math.sin(angle)
            sx, sy = (1 if dx > 0 else -1), (1 if dy > 0 else -1)
            ix, iy = initial_x, initial_y
            tx = (((ix + (sx > 0)) * r - x) / dx) if abs(dx) > 1e-14 else math.inf
            ty = (((iy + (sy > 0)) * r - y) / dy) if abs(dy) > 1e-14 else math.inf
            step_x = r / abs(dx) if abs(dx) > 1e-14 else math.inf
            step_y = r / abs(dy) if abs(dy) > 1e-14 else math.inf
            while True:
                distance = min(tx, ty)
                if distance >= maximum:
                    return maximum
                if abs(tx - ty) < 1e-12:
                    # Treat touching a blocked corner conservatively.
                    if blocked(ix + sx, iy) or blocked(ix, iy + sy):
                        return max(0.0, distance)
                    ix, iy = ix + sx, iy + sy
                    tx, ty = tx + step_x, ty + step_y
                elif tx < ty:
                    ix, tx = ix + sx, tx + step_x
                else:
                    iy, ty = iy + sy, ty + step_y
                if blocked(ix, iy):
                    return max(0.0, distance)

        yaw = pose.yaw - self.origin[2]
        return tuple(ray(yaw - math.pi + 2 * math.pi * i / count) for i in range(count))


@dataclass(frozen=True)
class MissionWorld:
    """Map-backed simulator world queried in the chosen arena's local frame."""

    slam: SlamMap
    arena: Arena

    @property
    def width(self) -> float:
        return self.arena.width

    @property
    def height(self) -> float:
        return self.arena.height

    def free(self, x: float, y: float, radius: float = 0.0) -> bool:
        if not all(math.isfinite(v) for v in (x, y, radius)) or radius < 0:
            return False
        return (self.arena.contains(x, y, radius)
                and self.slam.free(*self.arena.local_to_map(x, y), radius))

    def segment_free(self, a: Pose | Sequence[float], b: Pose | Sequence[float],
                     radius: float = 0.0) -> bool:
        ax, ay = _point(a)
        bx, by = _point(b)
        if not self.free(ax, ay, radius) or not self.free(bx, by, radius):
            return False
        return self.slam.segment_free(self.arena.local_to_map(ax, ay),
                                     self.arena.local_to_map(bx, by), radius)

    def scan(self, pose: Pose, count: int = 120, max_range: float = 3.5) -> tuple[float, ...]:
        x, y = self.arena.local_to_map(pose.x, pose.y)
        # The physical SLAM map supplies laser returns; arena selection is a
        # navigation boundary, not an invented opaque wall for the sensor.
        return self.slam.scan(Pose(x, y, pose.yaw + self.arena.yaw), count, max_range)

    def reachable(self, start: Pose | Sequence[float], goal: Pose | Sequence[float],
                  radius: float, resolution: float | None = None) -> bool:
        """Conservative grid connectivity check, including swept-link clearance.

        A negative result can miss corridors narrower than the search grid. It
        must not be described as proof that no continuous path exists.
        """
        sx, sy = _point(start)
        gx, gy = _point(goal)
        if not self.free(sx, sy, radius) or not self.free(gx, gy, radius):
            return False
        if self.segment_free((sx, sy), (gx, gy), radius):
            return True
        step = min(0.05, self.slam.resolution) if resolution is None else _number(resolution, "resolution")
        if step <= 0 or self.width * self.height / (step * step) > 250_000:
            raise ValueError("Connectivity grid must be positive and at most 250000 cells")
        nx, ny = math.ceil(self.width / step), math.ceil(self.height / step)
        cache: dict[tuple[int, int], bool] = {}

        def point(node: tuple[int, int]) -> tuple[float, float]:
            return (node[0] + 0.5) * step, (node[1] + 0.5) * step

        def valid(node: tuple[int, int]) -> bool:
            ix, iy = node
            if not (0 <= ix < nx and 0 <= iy < ny):
                return False
            if node not in cache:
                cache[node] = self.free(*point(node), radius)
            return cache[node]

        def nearby(x: float, y: float) -> set[tuple[int, int]]:
            ix, iy = math.floor(x / step), math.floor(y / step)
            return {(a, b) for a in range(ix - 1, ix + 2) for b in range(iy - 1, iy + 2)
                    if valid((a, b)) and self.segment_free((x, y), point((a, b)), radius)}

        begins, ends = nearby(sx, sy), nearby(gx, gy)
        if not begins or not ends:
            return False
        visited, pending = set(begins), deque(begins)
        neighbors = ((1, 0), (-1, 0), (0, 1), (0, -1),
                     (1, 1), (1, -1), (-1, 1), (-1, -1))
        while pending:
            node = pending.popleft()
            if node in ends:
                return True
            for dx, dy in neighbors:
                other = node[0] + dx, node[1] + dy
                if other in visited or not valid(other):
                    continue
                if self.segment_free(point(node), point(other), radius):
                    visited.add(other)
                    pending.append(other)
        return False
