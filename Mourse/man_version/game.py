"""AUT GAME - the game logic.

Rules implemented here:
  * the rat starts at the start point, the cheese waits at the finish point
  * the rat loops every computation time until it wins; nothing is computed
    before the run starts (each loop turn = think from memory + walk 1 unit)
  * when the rat has used more than the time limit (3 minutes) it dies
  * the clock only runs while the rat runs (Stop pauses it)

The Game object is step-based so the window buttons can drive it:
  reset_run() / start() / pause() / tick() / new_map() / save_map()
`run()` is the classic console loop built on top of those pieces.

Run separately:  python game.py   -> full game in the console (ascii mode)
"""
import time
from dataclasses import dataclass

from config import GameConfig
from graphics import Renderer, make_renderer
from rat import Rat
from stage import Stage, StageValidationError


@dataclass
class RunReport:
    status: str            # WIN / DEAD / CLOSED
    steps: int             # cells walked (1 unit each)
    bumps: int             # decisions that hit a wall (no move)
    decisions: int         # steps + bumps = computation times used
    elapsed: float         # computation time used (s)
    thinking: float        # pure thinking time inside that (s)
    backtracks: int        # walked steps in backtrack mode


class Game:
    def __init__(self, config: GameConfig | None = None,
                 renderer: Renderer | None = None):
        self.config = config or GameConfig()
        self.stage = Stage(self.config.stage)
        self.renderer = renderer or make_renderer(self.config)
        self.rat: Rat
        self.status = "READY"
        self._elapsed_acc = 0.0
        self._run_started: float | None = None
        self._sync_points()
        self.reset_run()

    # --------------------------------------------------------------- clock
    @property
    def elapsed(self) -> float:
        """Computation time used so far (the clock pauses with Stop)."""
        running = time.perf_counter() - self._run_started if self._run_started else 0.0
        return self._elapsed_acc + running

    def _freeze_clock(self) -> None:
        self._elapsed_acc = self.elapsed
        self._run_started = None

    # ------------------------------------------------------------- control
    def reset_run(self) -> None:
        """Fresh rat on the start point, empty memory, clock at zero."""
        self.rat = Rat(self.config.rat, self.config.stage.unit_cm)
        self.rat.place(self.stage.start)      # rat begins at the start point
        self.rat.sense(self.stage, self.stage.finish)
        self._elapsed_acc = 0.0
        self._run_started = None
        self.status = "READY"

    def start(self) -> None:
        """Start (or continue) the run. After WIN/DEAD it starts over."""
        if self.status in ("WIN", "DEAD"):
            self.reset_run()
        if self.status in ("READY", "PAUSED"):
            self._run_started = time.perf_counter()
            self.status = "RUNNING"

    def pause(self) -> None:
        if self.status == "RUNNING":
            self._freeze_clock()
            self.status = "PAUSED"

    def tick(self) -> None:
        """ONE computation time: check the clock, think from memory, walk 1 unit."""
        if self.status != "RUNNING":
            return
        if self.elapsed > self.config.time.limit_seconds:
            self.rat.alive = False            # more than 3 minutes -> dead
            self.status = "DEAD"
            self._freeze_clock()
            return
        self.rat.step(self.stage, self.stage.finish)
        if self.rat.pos == self.stage.finish:
            self.rat.won = True
            self.status = "WIN"
            self._freeze_clock()

    # ---------------------------------------------------------------- maps
    def _sync_points(self) -> None:
        """Keep the config mirroring the real points on the field."""
        self.config.stage.start_point = self.stage.start
        self.config.stage.finish_point = self.stage.finish

    def new_map(self, use_saved: bool) -> str:
        """Swap the field: saved map (True) or a fresh random one (False)."""
        self.config.stage.use_saved_map = use_saved
        if not use_saved:
            self.config.stage.seed = None     # a truly new maze every press
        try:
            self.stage = Stage(self.config.stage)
        except StageValidationError:
            # kept points don't fit the new maze -> back to the corners
            self.config.stage.start_point = None
            self.config.stage.finish_point = None
            self.stage = Stage(self.config.stage)
        self._sync_points()
        self.reset_run()
        return f"map: {self.stage.source}"

    def save_map(self, path: str | None = None) -> str:
        return f"map saved to {self.stage.save(path).name}"

    def load_map_file(self, path: str) -> str:
        """Load one specific map file (used by the Load Map file picker)."""
        self.config.stage.map_file = path
        return self.new_map(use_saved=True)

    # -------------------------------------------------------------- points
    def _move_point(self, start=None, finish=None) -> str:
        try:
            steps = self.stage.set_points(start=start, finish=finish)
        except StageValidationError as exc:
            return f"cannot place there: {exc}"
        self._sync_points()
        self.reset_run()
        what = (f"start moved to {self.stage.start}" if start is not None
                else f"cheese moved to {self.stage.finish}")
        return f"{what} - route needs {steps} computation times (rat reset)"

    def set_start(self, cell) -> str:
        return self._move_point(start=cell)

    def set_cheese(self, cell) -> str:
        return self._move_point(finish=cell)

    def random_points(self) -> str:
        self.stage.randomize_points()
        self._sync_points()
        self.reset_run()
        return (f"random tiles: start {self.stage.start}, "
                f"cheese {self.stage.finish} (rat reset)")

    def report(self) -> RunReport:
        return RunReport(self.status, self.rat.steps, self.rat.bumps,
                         self.rat.decisions, self.elapsed,
                         self.rat.thinking_time, self.rat.backtracks)

    # -------------------------------------------------------------- output
    def print_header(self) -> None:
        t = self.config.time
        print("=" * 62)
        print("AUT GAME : the rat and the cheese")
        print("=" * 62)
        print(self.stage.info_text())
        print(f"Rat   : {self.rat.size_text()} "
              f"(fits one {self.config.stage.unit_cm:g} cm unit), "
              f"walks {self.config.rat.units_per_tick} unit per computation time")
        print("Sense : smells the cheese through the walls "
              f"({self.rat.scent_cm:.1f} cm away); NO lidar - the eye sees "
              "only the wall it is facing, so it must turn to look")
        print(f"Time  : the rat can run for {t.limit_seconds:g} s "
              f"({t.limit_seconds / 60:g} min) - longer and it dies")
        print("-" * 62)

    def print_summary(self, report: RunReport) -> None:
        t = self.config.time
        print("-" * 62)
        if report.status == "WIN":
            print("RESULT: WIN - the rat reached the cheese!")
        elif report.status == "DEAD":
            print("RESULT: DEAD - the rat ran out of time, no cheese today")
        else:
            print("RESULT: window closed before the end")
        optimal = len(self.stage.shortest_path(self.stage.start,
                                               self.stage.finish)) - 1
        print(f"computation times    : {report.decisions} = "
              f"{report.steps} walks + {report.bumps} bumps "
              f"(perfect route = {optimal} walks)")
        print(f"explored / backtracks: {self.rat.explored} cells / "
              f"{report.backtracks} walk-back steps")
        print(f"computation time used: {report.elapsed:.2f} s "
              f"of {t.limit_seconds:g} s "
              f"(remaining {max(0.0, t.limit_seconds - report.elapsed):.2f} s)")
        print(f"pure thinking        : {report.thinking * 1000:.2f} ms")
        print("=" * 62)

    # ---------------------------------------------------------------- loop
    def run(self) -> RunReport:
        """Console flow: start immediately and loop until the end."""
        self.print_header()
        pace = self.config.time.tick_seconds
        self.renderer.render(self)            # frame 0: rat on the start point
        self.start()                          # the clock starts WITH the run
        while self.status == "RUNNING":
            if self.renderer.closed:
                self.status = "CLOSED"
                self._freeze_clock()
                break
            self.tick()                       # think from memory + walk 1 unit
            self.renderer.render(self)
            if pace > 0 and self.status == "RUNNING":
                time.sleep(pace)              # pace of one computation time
        report = self.report()
        self.print_summary(report)
        self.renderer.finish(self)
        return report


if __name__ == "__main__":
    cfg = GameConfig(renderer="ascii")        # console demo of the full game
    cfg.time.tick_seconds = 0.0
    Game(cfg).run()
