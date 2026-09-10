"""AUT GAME - the stage (maze field).

Rules implemented here:
  * maze field of width x height units (default 30 x 30), 16 cm per unit
  * exactly ONE start point and ONE finish point (the cheese is at finish)
  * start and finish are always connected (perfect maze -> every cell reachable)
  * the shortest start->finish route must need at least `min_path_steps`
    computation times (1 step = 1 computation time), default 5
  * the field can SAVE its map (stage/maps/*.json) and play it again later,
    or the random switch (use_saved_map=False) makes a new random map

Run separately (from the aut_game folder or from inside stage/):
    python stage/stage.py              -> random maze, validate, show
    python stage/stage.py --save       -> also save it to the map file
    python stage/stage.py --load       -> load and show the saved map
"""
import argparse
import json
import math
import pathlib
import random
import sys
from collections import defaultdict, deque

# so this file can run separately from its own folder
_ROOT = str(pathlib.Path(__file__).resolve().parent.parent)
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from config import StageConfig


Cell = tuple[int, int]  # (x, y) in units, (0, 0) = top-left


class StageValidationError(Exception):
    """The stage breaks one of the stage rules."""


class Stage:
    def __init__(self, cfg: StageConfig | None = None):
        self.cfg = cfg or StageConfig()
        self.width = self.cfg.width_units
        self.height = self.cfg.height_units
        # exactly one start and one finish point
        self.start: Cell = (0, 0)
        self.finish: Cell = (self.width - 1, self.height - 1)  # cheese lives here
        self._passages: dict[Cell, set[Cell]] = defaultdict(set)
        self.source = ""

        if self.cfg.use_saved_map:                 # the random switch is OFF
            path = self.map_path()
            if path.exists():
                self._load(path)                   # the file brings its own points
                self.source = f"saved map ({path.name})"
            else:
                print(f"[stage] no saved map at {path} -> random maze instead")
        if not self.source:                        # the random switch is ON
            rng = random.Random(self.cfg.seed)
            self._generate(rng)
            self.source = f"random (seed={self.cfg.seed})"
            if self.cfg.start_point is not None:
                self.start = tuple(self.cfg.start_point)
            if self.cfg.finish_point is not None:
                self.finish = tuple(self.cfg.finish_point)
            if self.cfg.random_points:
                self.randomize_points(rng)
        self.validate()

    # ------------------------------------------------------------- geometry
    def in_bounds(self, cell: Cell) -> bool:
        x, y = cell
        return 0 <= x < self.width and 0 <= y < self.height

    def _grid_neighbors(self, cell: Cell):
        x, y = cell
        for nxt in ((x, y - 1), (x + 1, y), (x, y + 1), (x - 1, y)):
            if self.in_bounds(nxt):
                yield nxt

    def _open_between(self, a: Cell, b: Cell) -> None:
        """Knock the wall down between neighbor cells a and b."""
        self._passages[a].add(b)
        self._passages[b].add(a)

    def is_open(self, a: Cell, b: Cell) -> bool:
        """True when there is NO wall between neighbor cells a and b."""
        return b in self._passages[a]

    def open_neighbors(self, cell: Cell):
        return self._passages[cell]

    def size_cm(self) -> tuple[float, float]:
        return self.width * self.cfg.unit_cm, self.height * self.cfg.unit_cm

    def straight_cm(self, a: Cell, b: Cell) -> float:
        """Straight-line distance in cm, measured THROUGH the walls.

        This is the cheese smell the rat can sense from anywhere; walls do
        not stop the smell, they only stop the rat.
        """
        return math.hypot(b[0] - a[0], b[1] - a[1]) * self.cfg.unit_cm

    # ----------------------------------------------------------- generation
    def _generate(self, rng: random.Random) -> None:
        """Iterative DFS backtracker -> a perfect maze.

        A perfect maze has exactly one route between any two cells, so the
        start and finish can always reach each other with no problem.
        """
        visited = {self.start}
        stack = [self.start]
        while stack:
            cell = stack[-1]
            candidates = [n for n in self._grid_neighbors(cell) if n not in visited]
            if not candidates:
                stack.pop()
                continue
            nxt = rng.choice(candidates)
            self._open_between(cell, nxt)
            visited.add(nxt)
            stack.append(nxt)

    # ------------------------------------------------------------ save/load
    def map_path(self, path: str | None = None) -> pathlib.Path:
        """Map files are relative to the stage folder unless absolute."""
        p = pathlib.Path(path or self.cfg.map_file)
        return p if p.is_absolute() else pathlib.Path(__file__).resolve().parent / p

    def save(self, path: str | None = None) -> pathlib.Path:
        """Save this map as JSON so the same field can be played again."""
        target = self.map_path(path)
        target.parent.mkdir(parents=True, exist_ok=True)
        data = {
            "width_units": self.width,
            "height_units": self.height,
            "start": list(self.start),
            "finish": list(self.finish),
            # one row per line, '1' = no wall in that direction
            "open_east": ["".join("1" if self.is_open((x, y), (x + 1, y)) else "0"
                                  for x in range(self.width))
                          for y in range(self.height)],
            "open_south": ["".join("1" if self.is_open((x, y), (x, y + 1)) else "0"
                                   for x in range(self.width))
                           for y in range(self.height)],
        }
        target.write_text(json.dumps(data, indent=1), encoding="utf-8")
        return target

    def _load(self, path: pathlib.Path) -> None:
        data = json.loads(path.read_text(encoding="utf-8"))
        self.width = int(data["width_units"])
        self.height = int(data["height_units"])
        self.start = tuple(data["start"])
        self.finish = tuple(data["finish"])
        self._passages = defaultdict(set)
        for y, row in enumerate(data["open_east"]):
            for x, flag in enumerate(row):
                if flag == "1":
                    self._open_between((x, y), (x + 1, y))
        for y, row in enumerate(data["open_south"]):
            for x, flag in enumerate(row):
                if flag == "1":
                    self._open_between((x, y), (x, y + 1))

    # ---------------------------------------------------------------- points
    def set_points(self, start: Cell | None = None,
                   finish: Cell | None = None) -> int:
        """Move the start and/or the cheese tile; every stage rule is
        re-checked and the old points come back if the new ones break one.
        Returns the shortest start->finish steps."""
        old = (self.start, self.finish)
        if start is not None:
            self.start = tuple(start)
        if finish is not None:
            self.finish = tuple(finish)
        try:
            return self.validate()
        except StageValidationError:
            self.start, self.finish = old
            raise

    def randomize_points(self, rng: random.Random | None = None) -> None:
        """Roll random start + cheese tiles that respect the rules."""
        rng = rng or random.Random()
        for _ in range(200):
            start = (rng.randrange(self.width), rng.randrange(self.height))
            finish = (rng.randrange(self.width), rng.randrange(self.height))
            try:
                self.set_points(start, finish)
                return
            except StageValidationError:
                continue
        raise StageValidationError("could not find valid random points")

    # ---------------------------------------------------------- pathfinding
    def shortest_path(self, src: Cell, dst: Cell) -> list[Cell] | None:
        """BFS shortest path (list of cells src..dst), None if unreachable.

        This is the "map knowledge" the rat uses: it can sense the cheese
        through the walls and plan the shortest legal route, but the route
        itself never crosses a wall.
        """
        prev: dict[Cell, Cell | None] = {src: None}
        queue = deque([src])
        while queue:
            cur = queue.popleft()
            if cur == dst:
                break
            for nxt in self._passages[cur]:
                if nxt not in prev:
                    prev[nxt] = cur
                    queue.append(nxt)
        if dst not in prev:
            return None
        path = []
        cell: Cell | None = dst
        while cell is not None:
            path.append(cell)
            cell = prev[cell]
        path.reverse()
        return path

    # ----------------------------------------------------------- validation
    def validate(self) -> int:
        """Check every stage rule; returns the shortest start->finish steps."""
        if not (self.in_bounds(self.start) and self.in_bounds(self.finish)):
            raise StageValidationError("start/finish outside the field")
        if self.start == self.finish:
            raise StageValidationError("start and finish must be two different points")
        path = self.shortest_path(self.start, self.finish)
        if path is None:
            raise StageValidationError("start and finish cannot reach each other")
        steps = len(path) - 1  # 1 step = 1 computation time
        if steps < self.cfg.min_path_steps:
            raise StageValidationError(
                f"start->finish needs only {steps} computation times, "
                f"rule requires at least {self.cfg.min_path_steps}"
            )
        return steps

    # -------------------------------------------------------------- display
    def ascii_art(self, rat_pos: Cell | None = None, trail: set[Cell] | None = None) -> str:
        """ASCII picture of the maze. S=start, C=cheese, R=rat, .=rat trail."""
        trail = trail or set()
        lines = []
        for y in range(self.height):
            top = "+"
            mid = ""
            for x in range(self.width):
                top += "  +" if self.is_open((x, y), (x, y - 1)) else "--+"
                left = " " if self.is_open((x, y), (x - 1, y)) else "|"
                cell = (x, y)
                if cell == rat_pos:
                    ch = "R"
                elif cell == self.finish:
                    ch = "C"
                elif cell == self.start:
                    ch = "S"
                elif cell in trail:
                    ch = "."
                else:
                    ch = " "
                mid += f"{left}{ch} "
            lines.append(top)
            lines.append(mid + "|")
        lines.append("+--" * self.width + "+")
        return "\n".join(lines)

    def info_text(self) -> str:
        w_cm, h_cm = self.size_cm()
        steps = len(self.shortest_path(self.start, self.finish)) - 1
        return (
            f"Stage : {self.width} x {self.height} units "
            f"(1 unit = {self.cfg.unit_cm:g} cm -> {w_cm:g} x {h_cm:g} cm)\n"
            f"Map   : {self.source}\n"
            f"Start : {self.start}   Finish (cheese): {self.finish}\n"
            f"Route : shortest start->finish = {steps} computation times "
            f"(rule: at least {self.cfg.min_path_steps})"
        )


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="AUT GAME stage (maze field)")
    ap.add_argument("--save", action="store_true",
                    help="save this maze to the map file")
    ap.add_argument("--load", action="store_true",
                    help="load the saved map instead of a random maze")
    ap.add_argument("--seed", type=int, default=None,
                    help="seed for the random maze")
    ap.add_argument("--random-points", action="store_true",
                    help="random start and cheese tiles")
    args = ap.parse_args()

    stage = Stage(StageConfig(seed=args.seed, use_saved_map=args.load,
                              random_points=args.random_points))
    if args.save:
        print(f"map saved to {stage.save()}")
    print(stage.info_text())
    print(stage.ascii_art(rat_pos=stage.start))
    print("stage OK - all rules validated")
