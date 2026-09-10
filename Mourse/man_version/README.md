# AUT GAME - Rat Maze

A rat wakes up at the start of a 30 x 30 maze (16 cm per unit). The cheese
waits at the finish. The rat has NO map: it only smells the cheese (straight
line in cm, through the walls) and sees the walls of the cell it stands in.
Every computation time it thinks with what it has learned so far and walks
exactly 1 unit - forward, backward, left or right. More than 3 minutes and
it dies.

## Play

```
python main.py                    # game window with buttons
python main.py --renderer ascii   # console mode (runs immediately)
python main.py --seed 7 --tick 0.02
```

Window buttons:

| button        | what it does                                        |
|---------------|-----------------------------------------------------|
| Start         | run the rat (after WIN/DEAD: same map, fresh rat)   |
| Stop          | pause - the 3-minute clock stops too                |
| Reset         | wipe the path and put the rat back at the start - the maze stays the same |
| Random Map    | roll a brand new random maze                        |
| Save Map      | file dialog: type any name, or keep the suggestion maze.json, maze1.json, maze2.json ... in `stage/maps/` |
| Load Map      | file dialog - pick any saved map and play it        |
| Set Start     | then click any tile on the maze to put the start    |
| Set Cheese    | then click any tile to put the cheese               |
| Random Points | roll random start + cheese tiles                    |

Moving a tile resets the rat (fresh memory) and every rule is re-checked -
an illegal spot (same tile, route shorter than 5 computation times) is
refused and the old tiles stay.

The same from the command line: `--save-map`, `--use-map`, `--map-file`,
`--start X Y`, `--cheese X Y`, `--random-points`.

## How the rat thinks (no map, no lidar!)

Full explanation for programmers: **[rat/ALGORITHM.md](rat/ALGORITHM.md)**.

The rat's only senses are its nose (cheese smell = straight-line cm,
through the walls) and ONE eye pointing straight ahead. It cannot see
around itself: to know whether a direction has a wall it must first TURN
to face it.

Every computation time the brain returns exactly one decision -
`forward` / `left` / `right` / `backward` - and the body executes it:
turn there, look, then walk 1 cell if open, or BUMP (stay, remember the
wall forever) if not. Choosing what to try is greedy by smell; dead ends
are escaped by walking back through remembered passages (BFS over memory,
recomputed every round - no stored plan). Walls are remembered from both
sides, and the rat never wastes a round on a side that cannot reveal a
new cell. A typical 30x30 run is ~560 rounds where a map-knowing rat
would walk the perfect ~300 (benchmarks in rat/ALGORITHM.md).

All the game rules, with sources and a compliance audit, are collected
in **[rule.md](rule.md)**. The brain is fully rule-legal: walls become
known only by facing them, and the rat does not know the arena size (it
learns the border by bumping like any other wall). rule.md also logs the
exhaustive search for a better legal algorithm and why each candidate
lost.

The code in `rat/rat.py` is split with hard borders so the algorithm is
obvious: `RatMemory` (wall facts + visited), `ExplorerBrain` (the pure
decision algorithm - it can never touch the maze), `Rat` (body: turn,
look, walk, bump). Swap the brain with
`Rat(cfg, unit_cm, brain=YourBrain())`.

## The colored trail

The path is painted by freshness: the newest steps are strong and dark,
older steps fade to light. The hue also rotates while the rat runs
(one full color cycle every 400 steps), so a long run is never squeezed
into the 255 shades of a single color. Both knobs live in
`config.TrailConfig`.

## Folder structure

```
aut_game/
|- main.py        entry point -> game window with buttons (or console)
|- config.py      every setting (Stage/Rat/Time/Trail/GameConfig)
|- game.py        step-based game: reset/start/pause/tick + console run()
|- graphics.py    TkRenderer (drawing), TkApp (buttons), AsciiRenderer
|- stage/         the maze field in its own folder
|  |- stage.py    generation, save/load, validation, smell distance
|  \- maps/       saved maps (JSON)
|- rat/           the player in its own folder
|  |- rat.py      PART 1 memory / PART 2 brain / PART 3 body
|  \- ALGORITHM.md  the algorithm explained for external programmers
|- rule.md        every game rule + sources + compliance audit
\- promt.md       prompt log (filled automatically by a Claude Code hook)
```

## Every file also runs on its own

| command                        | shows                                             |
|--------------------------------|---------------------------------------------------|
| `python config.py`             | the active configuration                          |
| `python stage/stage.py`        | a random maze + rule validation (`--save`/`--load`) |
| `python rat/rat.py`            | 8x8 demo: watch the rat explore to the cheese     |
| `python graphics.py`           | static infographic window (`--ascii` for console) |
| `python game.py`               | full game in the console                          |

(They also work started from inside their own folder, e.g. `cd rat && python rat.py`.)

## Rules -> code

| rule                                                | where                              |
|-----------------------------------------------------|------------------------------------|
| 30 x 30 units, 16 cm per unit                        | `config.StageConfig`               |
| one start, one finish, always connected              | `stage/stage.py` (perfect maze)    |
| start->finish needs at least 5 computation times     | `Stage.validate`                   |
| save map / load map / random switch                  | `Stage.save`, `Stage._load`, `Game.new_map` |
| start/cheese tiles: place by click or random         | `Stage.set_points`, `Stage.randomize_points` |
| rat max 16 x 16 cm (fits one unit)                   | `rat/rat.py` `Rat.__init__`        |
| 1 unit per computation time                          | `Rat.step`                         |
| dead after more than 3 minutes of running            | `Game.tick` (clock pauses on Stop) |
| smell in cm through walls, walking never through     | `Stage.straight_cm` + wall guard in `Rat.step` |
| no map - the way is learned only by running          | `Rat.sense/think/_backtrack_step`  |
| only forward / backward / left / right               | `Rat._action_for`                  |
| loop every computation time, no thinking before run  | `Game.tick` (thinking happens inside) |
| trail: new = strong color, old = light, hues rotate  | `TkRenderer._trail_color`, `config.TrailConfig` |

## Configure the future

All knobs live in `config.py`. Build your own config and pass it to `Game`:

```python
from config import GameConfig
from game import Game

cfg = GameConfig()
cfg.stage.use_saved_map = True   # random switch off -> play the saved map
cfg.trail.hue_cycle_steps = 200  # faster color rotation
cfg.time.tick_seconds = 0.0      # instant run (console)
Game(cfg).run()
```
