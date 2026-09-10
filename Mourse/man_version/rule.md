# rule.md - The Rules of AUT GAME

Compiled from every prompt in [promt.md](promt.md). When two prompts
conflict, the NEWER one wins. Each rule cites its source prompt.

## Stage rules

| id | rule | source (promt.md) |
|----|------|-------------------|
| S1 | maze field is 30 x 30 units | 2026-07-06 10:54 |
| S2 | 1 unit = 16 cm | 10:54 |
| S3 | exactly ONE start point and ONE finish point; the cheese is at the finish | 10:54 |
| S4 | start and finish can always reach each other | 10:54 |
| S5 | the shortest start->finish route needs at least 5 computation times | 10:54 |
| S6 | the field can save its map and play it again, or the random switch rolls a new one | 11:04 |
| S7 | start and cheese tiles are configurable at run time, or random | 12:33 |
| S8 | map files: name is configurable on save; default maze, maze1, maze2, ... | 13:03 + 13:05 |

## Rat rules

| id | rule | source |
|----|------|--------|
| R1 | the rat is not bigger than 16 x 16 cm (fits one unit) | 10:54 |
| R2 | the rat walks at most 1 unit per computation time | 10:54 |
| R3 | more than 3 minutes of computation time -> the rat is dead | 10:54 |
| R4 | the rat starts at the start point | 10:54 |
| R5 | moves are only forward / backward / left / right | 10:54 |
| R6 | the rat loops every computation time until it wins; it cannot think before the run | 10:54 |
| R7 | the distance the rat knows is rat->cheese in CM, straight line THROUGH walls; it can never walk through a wall | 2026-07-06 12:18 |
| R8 | the rat must NOT know the way in advance; the way can be known only from running | 12:18 ("make it can't know, the way to know is from when run only") |
| R9 | NO lidar: the rat cannot sense its surroundings; to know whether a direction has a wall it must first TURN to face it | 2026-07-18 12:06 |
| R10 | the decision each round is exactly one of forward / left / right / backward, one cell at most | 2026-07-18 12:06 |
| R11 | the closer-smelling direction is tried FIRST - no fixed left-before-right / compass pattern | 2026-07-18 12:14 |

Superseded: the original 10:54 line "rat can find shortest path to the
cheese through wall" was replaced by R8 + R9 (the rat may no longer know
the route, only the smell).

## Interpretation notes (how gray areas are ruled)

1. **Comparing the smell of the four neighbor cells is legal.** R11
   explicitly orders the rat to take the *closer* direction first, so the
   rules themselves grant directional smell (a gradient nose).
2. **Evaluating the smell of a far, remembered cell is legal.** The rat
   knows its own coordinates (dead reckoning) and has sampled the cheese
   distance from every cell it visited; from a few samples the cheese
   position follows by triangulation. That is *thinking about data
   gathered while running* - allowed by R8. It creates no wall knowledge,
   so R9 is untouched.
3. **The Stop button freezes the 3-minute clock.** R3 counts "computation
   time used"; a paused rat uses none.
4. **A bump is a legal round.** The rat moved 0 units (R2 says *at most*
   1) and learned the wall it was facing (R9).
5. **Choosing NOT to look is legal (the new-ground policy).** R9 limits
   how walls can become KNOWN; it does not force the rat to inspect
   every side. Skipping an UNKNOWN side that points at an
   already-visited cell claims no knowledge (the side stays UNKNOWN in
   memory) - the rat just declines a try that cannot reveal a new cell:
   a wall wastes a bump, an opening only re-enters ground it already
   knows. A safety net still faces those sides if nothing else is left,
   so no maze can deadlock. Related and also legal: wall facts are
   remembered from BOTH sides - if the rat at (20,10) once faced south,
   then standing at (20,9) it already knows its north side without
   turning (that is memory of something it DID face, not a deduction).

## Compliance audit (2026-07-18)

| rule | enforced by | status |
|------|-------------|--------|
| S1-S5 | `Stage.validate` (raises on any violation, also for loaded/edited maps) | OK |
| S6-S8 | `Stage.save/_load`, `Game.new_map`, save dialog | OK |
| R1 | `Rat.__init__` raises if bigger than one unit | OK |
| R2 | `Rat.step` walks exactly 0 or 1 cell per round | OK |
| R3 | `Game.tick` kills the rat past the limit | OK |
| R4 | `Game.reset_run` places the rat on the start | OK |
| R5/R10 | brain may only return the 4 actions; anything else raises `RatError` | OK |
| R6 | all thinking happens inside `step()` during the run, re-done every round | OK |
| R7 | smell = `Stage.straight_cm`; walking checks `look()`; a wall can never be crossed | OK |
| R8 | the brain's inputs are memory + smell only; the signature makes touching the maze impossible | OK |
| R9 | wall facts enter `RatMemory` in exactly one place: after turning and looking | **OK in "strict" mode (default)** - see the finding below |
| R11 | closest smell first; ties by smallest turn | OK |

### Finding history: the "best method" upgrade vs rule R9

The 12:21 upgrade ("find the best method") added two prunings - *skip
unknown sides toward visited cells* and *the arena edge is a wall*. The
first audit classified BOTH as R9 violations (knowing walls without
facing) and turned them off by default.

**Revised after a challenge (12:47 prompt, the (20,10)/(20,9) example):**
skipping unknown sides toward visited cells is a CHOICE, not a knowledge
claim (interpretation note 5) - it is legal and is now always on. The
arena-edge deduction is different: the rat was never granted the field
size by any prompt and cannot derive it from running until it bumps the
border, so treating the edge as known remains an R9 bend.

* `strict` (DEFAULT) - fully rule-legal: greedy by smell + tie-breaks
  + the new-ground policy. Walls become KNOWN only by facing.
* `smart` - additionally knew the arena edge without bumping it.
  **REMOVED on 2026-07-18**: the player confirmed the rat cannot know
  the field size ("it does not know where it will spawn"), so this
  deduction has no legal basis. Only the legal brain remains.

## Best known algorithm (exhaustive legal search, 2026-07-18)

Everything below was measured on identical maze sets (up to 100
corner-to-corner + 100 random-tile fields), mean rounds to the cheese -
full method in [rat/ALGORITHM.md](rat/ALGORITHM.md):

| brain | corner | random tiles | verdict |
|-------|--------|--------------|---------|
| **greedy + two-sided memory + new-ground policy (CURRENT)** | **577.7** | **912.1** | champion |
| greedy without the new-ground policy | +11% | +9% | strictly worse |
| A*-style planner, any smell weight <= 1 | = greedy exactly | = greedy | collapses into greedy |
| A*-style planner, smell weight 2-3 | ~2x worse | ~15% worse | smell through walls misleads |
| manhattan distance instead of straight-line | -1% | **+18%** | inconsistent, rejected |
| bump-risk penalty on unknown sides (0.25-1 unit, incl. adaptive) | +-0% | +-0% | structurally inert* |
| forward-momentum bonus beyond ties (2-16 cm) | -0.3% | -1.5% | ~50/50 head-to-head = noise |
| arena edge assumed known | -0.4% | -3% | ILLEGAL (removed) |

*Risk penalties can never fire: looking and walking are fused in one
round, so the rat never holds a "known-open door to unvisited ground" to
compare an unknown side against.

Why the champion is likely the legal optimum in practice: the smell
changes by at most 1 unit per step, so "walk-cost + smell" planning
provably collapses into greedy; the smell passes through walls, so
weighting it harder always backfires; two-sided memory plus the
new-ground policy already extract every fact and every useful *choice* a
run can give. The only lever left is probabilistic guessing about maze
structure, and every guessing variant measured to date is either inert,
inconsistent between field layouts, or statistical noise.
