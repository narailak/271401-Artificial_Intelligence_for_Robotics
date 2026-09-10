"""AUT GAME - the rat (player).

The full algorithm explanation for external programmers is in
rat/ALGORITHM.md. Short version:

WHAT THE RAT CAN SENSE (it has NO map and NO lidar):
  * nose : cheese smell = straight-line distance in cm, through the walls
  * eye  : sees ONLY the wall segment it is FACING. To know whether a
           direction has a wall, the rat must first TURN to face it.

DECISION PER COMPUTATION TIME (returned by the brain):
  one of  "forward" / "left" / "right" / "backward"  - one cell at most.
  The body turns to that direction, looks, and:
      open ahead -> walks exactly 1 unit
      wall ahead -> BUMP: stays in place, the round is spent,
                    but the wall is now remembered forever.

THE CODE IS SPLIT IN 3 CLEARLY SEPARATED PARTS (same file, hard borders):
  PART 1  RatMemory     - what the rat remembers (wall facts + visited)
  PART 2  ExplorerBrain - the decision algorithm. Pure logic: it receives
                          ONLY memory + position + heading + a smell
                          function. It can never touch the maze.
  PART 3  Rat (body)    - sensors and movement. The ONLY code that talks
                          to the Stage, and only through look() and smell.

Swap the algorithm by passing another brain: Rat(cfg, unit_cm, brain=MyBrain())

Run separately (from the aut_game folder or from inside rat/):
    python rat/rat.py   -> small 8x8 demo, watch decisions + bumps
"""
import pathlib
import sys
import time
from collections import deque

# so this file can run separately from its own folder
_ROOT = str(pathlib.Path(__file__).resolve().parent.parent)
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from config import RatConfig, StageConfig
from stage import Cell, Stage


class RatError(Exception):
    pass


# ---------------------------------------------------------------------------
# Shared direction vocabulary (module level so brain and body speak the same)
# ---------------------------------------------------------------------------
DIRS = {"N": (0, -1), "E": (1, 0), "S": (0, 1), "W": (-1, 0)}  # y grows down
ARROW = {"N": "^", "E": ">", "S": "v", "W": "<"}
LEFT_OF = {"N": "W", "W": "S", "S": "E", "E": "N"}
RIGHT_OF = {"N": "E", "E": "S", "S": "W", "W": "N"}
OPPOSITE = {"N": "S", "S": "N", "E": "W", "W": "E"}

ACTIONS = ("forward", "left", "right", "backward")

# wall knowledge states
UNKNOWN, OPEN, WALL = "unknown", "open", "wall"


def neighbor(cell: Cell, direction: str) -> Cell:
    dx, dy = DIRS[direction]
    return (cell[0] + dx, cell[1] + dy)


def direction_of(heading: str, action: str) -> str:
    """Which absolute direction does this relative action point to?"""
    return {"forward": heading, "left": LEFT_OF[heading],
            "right": RIGHT_OF[heading], "backward": OPPOSITE[heading]}[action]


def action_for(heading: str, direction: str) -> str:
    """The relative action that would face this absolute direction."""
    if direction == heading:
        return "forward"
    if direction == OPPOSITE[heading]:
        return "backward"
    return "left" if direction == LEFT_OF[heading] else "right"


# ===========================================================================
# PART 1: MEMORY - what the rat remembers (nothing else exists for it)
# ===========================================================================
class RatMemory:
    """Everything the rat has learned by running.

    * walls   : {(cell, direction) -> OPEN or WALL}. Anything not stored is
                UNKNOWN - the rat has simply never faced that way from there.
                When a wall fact is learned it is stored from both sides.
    * visited : the cells the rat has stood in.
    """

    def __init__(self):
        self.walls: dict[tuple[Cell, str], str] = {}
        self.visited: set[Cell] = set()

    # ------------------------------------------------------------ learning
    def mark_visited(self, cell: Cell) -> None:
        self.visited.add(cell)

    def learn_wall(self, cell: Cell, direction: str, is_open: bool,
                   other_side: Cell | None) -> None:
        state = OPEN if is_open else WALL
        self.walls[(cell, direction)] = state
        if other_side is not None:            # the same fact seen from the far side
            self.walls[(other_side, OPPOSITE[direction])] = state

    # ------------------------------------------------------------ querying
    def wall_state(self, cell: Cell, direction: str) -> str:
        return self.walls.get((cell, direction), UNKNOWN)

    def known_open_dirs(self, cell: Cell) -> list[str]:
        return [d for d in DIRS if self.wall_state(cell, d) == OPEN]

    def frontier_dirs(self, cell: Cell) -> list[str]:
        """Directions still worth trying from this cell:
        UNKNOWN (never faced it) or OPEN into a cell not visited yet."""
        out = []
        for d in DIRS:
            state = self.wall_state(cell, d)
            if state == UNKNOWN:
                out.append(d)
            elif state == OPEN and neighbor(cell, d) not in self.visited:
                out.append(d)
        return out


# ===========================================================================
# PART 2: BRAIN - the decision algorithm (pure logic, never sees the maze)
# ===========================================================================
class ExplorerBrain:
    """Returns ONE decision per computation time: forward/left/right/backward.

    Inputs (all of them - there is nothing hidden):
        memory  : RatMemory (wall facts + visited cells)
        pos     : where the rat stands
        heading : where the rat faces
        smell   : function cell -> cheese smell in cm (straight line,
                  through walls). This is geometry from the nose, not a map.

    THE ALGORITHM, step by step (see rat/ALGORITHM.md for the full story):

      0. NEW-GROUND POLICY (always on, fully legal): skip an UNKNOWN
         side that points at an already-visited cell. Trying it can
         never reveal a new cell - a wall costs a bump, an opening
         only re-enters known ground - so the rat CHOOSES not to
         look. No knowledge is claimed (the side stays UNKNOWN in
         memory), and rule R9 only limits knowing, not choosing.
         Safety net: if nothing can reveal new ground, those sides
         are then really faced. (The rat does NOT know the arena
         size - it can spawn anywhere - so border sides are learned
         by bumping like any other wall.)

      1. EXPLORE. List this cell's frontier directions =
             UNKNOWN  (never faced -> maybe a corridor, maybe a wall)
           + OPEN into a cell not visited yet (a known door to new ground).
         If the list is not empty: pick the direction whose next cell
         smells most of cheese - the CLOSER direction always goes first.
         Only on an exact smell tie the smaller turn wins (forward, then
         left/right, then backward); there is no fixed compass order.
         Facing it is how the rat finds out: walk through, or bump + learn.

      2. BACKTRACK. No frontier here: every direction is a known wall,
         a known door to already-visited ground, or exhausted. Do a BFS
         over REMEMBERED open passages (visited cells only) to the NEAREST
         cells that still have a frontier; among equally-near ones walk
         toward the best-smelling one. Return the first step of that
         remembered path. Recomputed every round - no stored plan.

      3. DONE. No frontier anywhere in memory: return "stay"
         (in a perfect maze this cannot happen before the cheese is found).

    Termination: every round either (a) learns a new wall fact, (b) enters
    a never-visited cell, or (c) walks one step of a shortest remembered
    path toward a cell where (a)/(b) is possible. All three are finite.
    """

    # how expensive a relative action feels; used ONLY to break smell ties
    TURN_COST = {"forward": 0, "left": 1, "right": 1, "backward": 2}

    def __init__(self):
        self.mode = "explore"      # "explore" / "backtrack" / "done" (for the HUD)
        # Always-on LEGAL policy (rule.md, interpretation note 5): never
        # spend a round on a side that cannot reveal a NEW cell - an
        # UNKNOWN side toward an already-visited cell is skipped. This
        # claims no wall knowledge (the side stays UNKNOWN in memory);
        # the rat simply chooses not to look there. Choosing not to look
        # breaks no rule, and a safety net still tries those sides when
        # nothing else is left.
        self.new_ground_first = True

    # ----------------------------------------------------- option pruning
    def _options(self, memory: RatMemory, cell: Cell,
                 new_ground_only: bool) -> list[str]:
        """Directions still worth trying from `cell`.

        * with `new_ground_only` (the always-on legal policy): skip an
          UNKNOWN side toward an already-visited cell - trying it can not
          reveal a new cell whatever the outcome, so the rat chooses not
          to look. No knowledge is claimed: the side stays UNKNOWN.

        The rat does NOT know the arena size (it can spawn anywhere), so
        sides pointing outside the field are tried like any other and
        learned as walls by bumping - rule R9 leaves no shortcut here."""
        out = []
        for d in DIRS:
            n = neighbor(cell, d)
            state = memory.wall_state(cell, d)
            if state == OPEN and n not in memory.visited:
                out.append(d)
            elif state == UNKNOWN and not (new_ground_only
                                           and n in memory.visited):
                out.append(d)
        return out

    def decide(self, memory: RatMemory, pos: Cell, heading: str, smell) -> str:
        action = self._decide(memory, pos, heading, smell,
                              new_ground_only=self.new_ground_first)
        if action is None and self.new_ground_first:
            # safety net: nothing can reveal new ground (possible only on
            # exotic hand-made maps) -> now really look at the skipped sides
            action = self._decide(memory, pos, heading, smell,
                                  new_ground_only=False)
        if action is None:
            # -- step 3: DONE -----------------------------------------------
            self.mode = "done"
            return "stay"
        return action

    def _decide(self, memory: RatMemory, pos: Cell, heading: str,
                smell, new_ground_only: bool) -> str | None:
        # -- step 1: EXPLORE ------------------------------------------------
        frontier = self._options(memory, pos, new_ground_only)
        if frontier:
            self.mode = "explore"
            # closest-smelling direction FIRST. Only when two directions
            # smell exactly the same does the rat prefer the smaller turn
            # (forward, then left/right, then backward) - never a fixed
            # compass order.
            best = min(frontier, key=lambda d: (
                smell(neighbor(pos, d)),
                self.TURN_COST[action_for(heading, d)]))
            return action_for(heading, best)
        # -- step 2: BACKTRACK ---------------------------------------------
        back = self._first_step_back(memory, pos, smell, new_ground_only)
        if back is not None:
            self.mode = "backtrack"
            return action_for(heading, back)
        return None

    def _first_step_back(self, memory: RatMemory, pos: Cell,
                         smell, new_ground_only: bool) -> str | None:
        """BFS through remembered open passages between visited cells to the
        NEAREST cells that still have a frontier; when several are equally
        near, walk toward the one that smells most of cheese. Returns the
        direction of the FIRST step of that remembered path."""
        prev: dict[Cell, Cell | None] = {pos: None}
        depth = {pos: 0}
        queue = deque([pos])
        candidates: list[Cell] = []
        found_depth: int | None = None
        while queue:
            cur = queue.popleft()
            if found_depth is not None and depth[cur] > found_depth:
                break                          # deeper than the first hits
            if cur != pos and self._options(memory, cur, new_ground_only):
                candidates.append(cur)
                found_depth = depth[cur]
                continue
            for d in memory.known_open_dirs(cur):
                nxt = neighbor(cur, d)
                if nxt in memory.visited and nxt not in prev:
                    prev[nxt] = cur
                    depth[nxt] = depth[cur] + 1
                    queue.append(nxt)
        if not candidates:
            return None
        target = min(candidates, key=smell)    # nearest first, then best smell
        cell = target                          # walk the chain back to pos
        while prev[cell] is not pos:
            cell = prev[cell]
        for d in DIRS:                         # cell is adjacent to pos
            if neighbor(pos, d) == cell:
                return d
        raise RatError("backtrack produced a non-adjacent step")  # impossible


# ===========================================================================
# PART 3: BODY - sensors and movement (the only part that touches the Stage)
# ===========================================================================
class Rat:
    """The physical rat: 16 x 16 cm, one eye pointing straight ahead.

    One computation time = step():
        1. sense   - stand in the cell (mark visited) + smell the cheese
        2. decide  - ask the brain: forward / left / right / backward
        3. turn    - rotate to face that direction (the eye turns with it)
        4. look    - NOW the facing wall becomes visible -> store in memory
        5. walk 1 unit if open, or BUMP (stay, count it) if it is a wall
    """
    DIRS = DIRS      # exposed for the renderers
    ARROW = ARROW

    def __init__(self, cfg: RatConfig | None = None, unit_cm: float = 16.0,
                 brain: ExplorerBrain | None = None):
        self.cfg = cfg or RatConfig()
        if self.cfg.width_cm > unit_cm or self.cfg.height_cm > unit_cm:
            raise RatError(
                f"rat {self.cfg.width_cm:g} x {self.cfg.height_cm:g} cm is bigger "
                f"than one {unit_cm:g} cm unit - not allowed"
            )
        self.brain = brain or ExplorerBrain()
        self.memory = RatMemory()
        self.pos: Cell = (0, 0)
        self.heading = "E"
        self.steps = 0            # cells actually walked (1 unit each)
        self.bumps = 0            # decisions that hit a wall (no move)
        self.decisions = 0        # steps + bumps = computation times used
        self.backtracks = 0       # walked steps in backtrack mode
        self.alive = True
        self.won = False
        self.last_action = "-"
        self.scent_cm = 0.0
        self.thinking_time = 0.0
        self.trail: list[Cell] = []

    # --------------------------------------------------------------- setup
    def place(self, cell: Cell) -> None:
        """Put the rat on the start point with an empty memory."""
        self.pos = cell
        self.trail = [cell]
        self.memory = RatMemory()

    def size_text(self) -> str:
        return f"{self.cfg.width_cm:g} x {self.cfg.height_cm:g} cm"

    # ------------------------------------------------------------- sensors
    def sense(self, stage: Stage, goal: Cell) -> None:
        """Nose only: standing here + the cheese smell. No walls yet!"""
        self.memory.mark_visited(self.pos)
        self.scent_cm = stage.straight_cm(self.pos, goal)

    def look(self, stage: Stage) -> bool:
        """Eye only: is the way OPEN straight ahead? (facing direction only)"""
        ahead = neighbor(self.pos, self.heading)
        return stage.in_bounds(ahead) and stage.is_open(self.pos, ahead)

    # ------------------------------------------------------------- walking
    def step(self, stage: Stage, goal: Cell) -> str:
        """One computation time. Returns the decision that was executed."""
        for _ in range(self.cfg.units_per_tick):
            self.sense(stage, goal)

            t0 = time.perf_counter()
            smell = lambda cell: stage.straight_cm(cell, goal)  # nose geometry
            action = self.brain.decide(self.memory, self.pos, self.heading, smell)
            self.thinking_time += time.perf_counter() - t0

            if action == "stay":
                self.last_action = "stay"
                return self.last_action
            if action not in ACTIONS:
                raise RatError(f"brain returned illegal decision {action!r}")
            self.decisions += 1

            self.heading = direction_of(self.heading, action)   # 3. turn
            ahead = neighbor(self.pos, self.heading)
            is_open = self.look(stage)                          # 4. look
            self.memory.learn_wall(self.pos, self.heading, is_open,
                                   ahead if stage.in_bounds(ahead) else None)

            if is_open:                                         # 5. walk ...
                if self.brain.mode == "backtrack":
                    self.backtracks += 1
                self.pos = ahead
                self.steps += 1
                self.trail.append(ahead)
                self.last_action = action
            else:                                               # ... or bump
                self.bumps += 1
                self.last_action = f"{action} (bump)"
        return self.last_action

    def pos_cm(self, unit_cm: float) -> tuple[float, float]:
        return self.pos[0] * unit_cm, self.pos[1] * unit_cm

    @property
    def explored(self) -> int:
        return len(self.memory.visited)


if __name__ == "__main__":
    stage = Stage(StageConfig(width_units=8, height_units=8, seed=42))
    rat = Rat(RatConfig(), unit_cm=stage.cfg.unit_cm)
    rat.place(stage.start)
    rat.sense(stage, stage.finish)
    print(f"demo: 8 x 8 maze, rat {rat.size_text()} starts at {stage.start}, "
          f"cheese at {stage.finish}, smell {rat.scent_cm:.1f} cm away")
    print("the rat has no lidar: it must TURN to a direction to see that wall")
    while rat.pos != stage.finish:
        before, before_heading = rat.pos, rat.heading
        decision = rat.step(stage, stage.finish)
        if rat.decisions <= 14 or rat.pos == stage.finish:
            outcome = "stays" if before == rat.pos else f"-> {rat.pos}"
            print(f"round {rat.decisions:>3}: decision={decision:<16} "
                  f"faces {rat.heading} ({ARROW[rat.heading]}) {outcome}, "
                  f"smell {rat.scent_cm:6.1f} cm")
        elif rat.decisions == 15:
            print("... exploring ...")
    optimal = len(stage.shortest_path(stage.start, stage.finish)) - 1
    print(f"cheese found: {rat.decisions} computation times = "
          f"{rat.steps} walks + {rat.bumps} bumps "
          f"(optimal walk {optimal}), explored {rat.explored} cells, "
          f"{rat.backtracks} walk-back steps")
    print(stage.ascii_art(rat_pos=rat.pos, trail=set(rat.trail)))
