# The Rat's Algorithm

This document explains, for any programmer reading the code for the first
time, exactly how the rat finds the cheese. Everything described here lives
in `rat/rat.py`, split into three parts with hard borders.

## 1. The world from the rat's point of view

The rat has **no map and no lidar**. Its complete sensor list:

| sensor | what it gives                                                   |
|--------|-----------------------------------------------------------------|
| nose   | cheese smell = straight-line distance in **cm**, measured **through** walls (walls stop the rat, not the smell) |
| eye    | ONE bit: is the wall segment **directly in front** open or closed. The eye points where the rat faces - to inspect another direction the rat must **turn** first |
| feet   | walking works or it doesn't (a bump is how a wall is discovered) |

It also knows its own position and heading (dead reckoning), and it never
forgets anything it has learned.

## 2. The decision, one per computation time

Every computation time the brain returns exactly one decision:

```
"forward" | "left" | "right" | "backward"     (relative to current heading)
```

The body then executes the round:

```
turn to the chosen direction        (the eye now points there)
look: open or wall?                 -> written into memory, both sides
open  -> walk exactly 1 cell
wall  -> BUMP: stay in place; the round is spent, the wall is now known
```

So "knowledge costs rounds": checking a direction and being wrong wastes
one computation time. That is intentional - it is the price of having no
lidar.

## 3. Memory (`RatMemory`)

```
walls   : {(cell, direction) -> OPEN | WALL}    # absent = UNKNOWN
visited : {cells the rat has stood in}
```

Two derived queries used by the brain:

* `known_open_dirs(cell)` - directions proven open from `cell`.
* `frontier_dirs(cell)`   - directions still worth trying from `cell`:
  * `UNKNOWN` - never faced that way (could be a corridor), or
  * `OPEN` into a cell **not visited yet** (a known door to new ground).

## 4. The brain (`ExplorerBrain.decide`) - the actual algorithm

Called once per computation time with `(memory, pos, heading, smell)`,
where `smell(cell)` returns the cheese smell at a cell (pure geometry from
the nose). The brain has **no access to the Stage object** - the function
signature makes cheating impossible.

```
STEP 0 - FREE KNOWLEDGE (inference, costs no rounds)
    two facts prune tries before any round is spent:
    a) arena edge: the field size is written in the game rules, so any
       direction leaving the field is a wall - never try it
    b) no-loop rule: a perfect maze has no cycles, so an UNKNOWN side
       pointing at an ALREADY-VISITED cell must be a wall (an opening
       there would create a second route = a loop) - never try it
    safety net: if (b) ever leaves nothing to try anywhere (possible only
    on a hand-made map WITH loops), the round is retried with (b) off

STEP 1 - EXPLORE
    F = memory.frontier_dirs(pos)
    if F is not empty:
        pick d in F whose neighbor cell smells most of cheese
            -> the CLOSER direction is always tried first
        tie-break (exactly equal smell only): the smaller turn wins
            forward (0) < left/right (1) < backward (2)
            - never a fixed compass or left-before-right order
        return relative_action(heading, d)

STEP 2 - BACKTRACK
    (this cell is exhausted: all four directions are known walls or
     known doors into already-visited ground)
    BFS over REMEMBERED open passages, visited cells only,
        from pos to the NEAREST cells with frontier_dirs != empty;
        among equally-near candidates pick the best-smelling one
    return the relative action of the FIRST step of that path
    (recomputed from scratch every round - the rat stores no plan)

STEP 3 - DONE
    no frontier anywhere in memory -> return "stay"
    (cannot happen before the cheese is found: the maze is connected)
```

### Why this is greedy but safe

The smell is only a *heuristic*: it pulls the rat toward the cheese but a
wall can be in between (the smell goes through walls). Wrong pulls cost
bumps and dead ends - and every one of them is written into memory, so a
mistake is never repeated. Exploration is therefore systematic (a DFS-like
sweep) with the smell only deciding the *order* in which branches are
tried.

### Why it always terminates

Each round does at least one of:

1. learn a NEW wall fact (a bump, or a first look at an opening),
2. enter a NEVER-visited cell,
3. walk one step of a shortest remembered path toward a cell where
   1 or 2 is possible.

(1) is bounded by 2 x number-of-wall-segments, (2) by the number of cells,
and (3) always makes progress toward a strictly closer frontier cell, so
the total number of rounds is finite. The maze is connected (perfect
maze), so the cheese cell is visited eventually - usually far sooner,
because the smell orders the search well.

### Measured: why THIS combination

Tournament over the same mazes (40 corner-to-corner + 40 random-point
fields, mean rounds to the cheese):

| brain                                   | corner | random points |
|-----------------------------------------|--------|---------------|
| **greedy + new-ground policy (DEFAULT)** | **562.5** | **779.6** |
| greedy without the new-ground policy    | 627.8  | 854.9         |
| A*-style planner, smell weight w<=1     | = greedy exactly | = greedy |
| A*-style planner, w=2..3                | 1197-1229 | 1017-1047  |
| manhattan instead of straight-line smell | 557.7 | 919.1 (rejected) |
| bump-risk penalties / adaptive risk     | no effect* | no effect*  |
| forward-momentum bonus beyond ties      | noise  | noise         |

*Risk penalties can never fire: look and walk are fused in one round, so
a "known-open door to unvisited ground" never exists to compare against.

NOTE: the rule audit in ../rule.md ruled the new-ground policy LEGAL
(choosing not to look claims no knowledge), so it is always on. The
former "smart" mode (arena edge known without bumping) was REMOVED:
the rat can spawn anywhere and has no legal way to know the field size.

* the two inference rules cut rounds ~11% and bumps ~25-29% for free;
* a full A*-style "score every remembered option by walk-cost + smell"
  planner is EQUAL to greedy at w=1 (the smell is 1-Lipschitz per step,
  so a nearer-by-(g+h) option is always found next door) and much WORSE
  when the smell is weighted up - smell passes through walls, so trusting
  it hard drags the rat into long detours it must pay back on foot.

Greedy + inference is therefore the best known method within these rules,
and it stays simple to explain.

## 5. Code map

| part                    | class           | talks to the maze?                  |
|-------------------------|-----------------|-------------------------------------|
| PART 1 memory           | `RatMemory`     | never                               |
| PART 2 algorithm        | `ExplorerBrain` | never (only memory + smell function)|
| PART 3 body             | `Rat`           | ONLY via `look()` (facing wall) and `stage.straight_cm` (smell) |

Direction vocabulary shared by all parts (module level):
`DIRS, LEFT_OF, RIGHT_OF, OPPOSITE, neighbor(), direction_of(), action_for()`.

## 6. Swapping the algorithm

Write any class with `decide(memory, pos, heading, smell) -> action`
and hand it to the body:

```python
from rat import Rat
rat = Rat(cfg.rat, unit_cm=16.0, brain=MyCleverBrain())
```

The body enforces the physics either way: illegal decision strings raise,
walls can never be walked through, one cell per computation time maximum.
