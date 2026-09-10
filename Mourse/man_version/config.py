"""AUT GAME - configuration.

All game rules/sizes live here as dataclasses so the game is easy to
re-configure later (change maze size, rat size, time limit, speed, ...)
without touching the game logic.

Run separately:  python config.py   -> prints the active configuration
"""
from dataclasses import dataclass, field


@dataclass
class StageConfig:
    """The maze field."""
    width_units: int = 30          # maze is 30 x 30 units
    height_units: int = 30
    unit_cm: float = 16.0          # 1 unit = 16 cm
    min_path_steps: int = 5        # start->finish must need at least 5 computation times
    seed: int | None = None        # None = a new random maze every run
    # the random switch: False -> random maze, True -> play the saved map
    use_saved_map: bool = False
    map_file: str = "maps/maze.json"   # saved maps live in the stage folder
    # where the rat starts and where the cheese waits:
    start_point: tuple[int, int] | None = None    # None = corner (0, 0)
    finish_point: tuple[int, int] | None = None   # None = opposite corner
    random_points: bool = False    # True = roll random start + cheese tiles


@dataclass
class RatConfig:
    """The player."""
    width_cm: float = 16.0         # rat must NOT be bigger than 16 x 16 cm
    height_cm: float = 16.0
    units_per_tick: int = 1        # rat walks only 1 unit per computation time


@dataclass
class TimeConfig:
    """Computation-time rules."""
    limit_seconds: float = 180.0   # rat dies when it uses more than this time (sec)
    tick_seconds: float = 0.05     # real duration of one computation time (animation pace)


@dataclass
class TrailConfig:
    """Colors of the path the rat has run.

    The newest steps get the strongest (darkest) color and older steps fade
    to light. The hue also rotates along the way, so the trail is not stuck
    inside the 255 shades of a single color.
    """
    fade_steps: int = 300          # how many recent steps keep a strong color
    hue_cycle_steps: int = 400     # steps for one full rotation through the hues


@dataclass
class GameConfig:
    stage: StageConfig = field(default_factory=StageConfig)
    rat: RatConfig = field(default_factory=RatConfig)
    time: TimeConfig = field(default_factory=TimeConfig)
    trail: TrailConfig = field(default_factory=TrailConfig)
    renderer: str = "tk"           # "tk" = infographic window, "ascii" = console only
    cell_px: int = 20              # pixel size of one unit in the tk window
    log_every_steps: int = 20      # console progress line every N computation times


def describe(cfg: GameConfig) -> str:
    s, r, t = cfg.stage, cfg.rat, cfg.time
    field_w_cm = s.width_units * s.unit_cm
    field_h_cm = s.height_units * s.unit_cm
    map_mode = f"saved map ({s.map_file})" if s.use_saved_map else f"random (seed={s.seed})"
    if s.random_points:
        points = "random tiles"
    else:
        points = (f"start={s.start_point or 'corner (0, 0)'}, "
                  f"cheese={s.finish_point or 'opposite corner'}")
    lines = [
        "AUT GAME configuration",
        "----------------------",
        f"Stage : {s.width_units} x {s.height_units} units, 1 unit = {s.unit_cm:g} cm",
        f"        -> field size {field_w_cm:g} x {field_h_cm:g} cm",
        f"        start->finish must take at least {s.min_path_steps} computation times",
        f"        map: {map_mode}",
        f"        points: {points}",
        f"Rat   : {r.width_cm:g} x {r.height_cm:g} cm, walks {r.units_per_tick} unit per computation time",
        f"Time  : rat can run up to {t.limit_seconds:g} s "
        f"({t.limit_seconds / 60:g} min), one computation time = {t.tick_seconds:g} s",
        f"Trail : strong color for the last {cfg.trail.fade_steps} steps, "
        f"hue rotates every {cfg.trail.hue_cycle_steps} steps",
        f"View  : renderer={cfg.renderer}, cell={cfg.cell_px}px, log every {cfg.log_every_steps} steps",
    ]
    return "\n".join(lines)


if __name__ == "__main__":
    print(describe(GameConfig()))
