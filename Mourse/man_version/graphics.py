"""AUT GAME - info graphic of the rat and the stage.

Renderers (pick with GameConfig.renderer):
  * TkRenderer    - live infographic window: maze, rat, cheese, colored trail
                    and a side panel with every stage/rat/time fact
  * AsciiRenderer - console fallback, prints the maze and progress lines
  * TkApp         - the playable window on top of TkRenderer with buttons:
                    Start / Stop / Random map / Save map / Load map

Trail colors: the newest steps are painted strong and dark, older steps
fade toward light. The hue also rotates while the rat runs, so a long run
is not limited to the 255 shades of one single color.

Run separately:  python graphics.py           -> static infographic window
                 python graphics.py --ascii   -> static console infographic
"""
import colorsys
import sys

from config import GameConfig


class Renderer:
    """Interface used by game.Game."""
    closed = False

    def render(self, game) -> None: ...
    def finish(self, game) -> None: ...


def hud_text(game) -> str:
    """The infographic facts shared by both renderers."""
    cfg = game.config
    stage, rat = game.stage, game.rat
    w_cm, h_cm = stage.size_cm()
    x_cm, y_cm = rat.pos_cm(cfg.stage.unit_cm)
    remaining = max(0.0, cfg.time.limit_seconds - game.elapsed)
    return "\n".join([
        "AUT GAME - RAT MAZE",
        "===================",
        "",
        "STAGE",
        f"  field    : {stage.width} x {stage.height} units",
        f"  unit     : {cfg.stage.unit_cm:g} cm",
        f"  size     : {w_cm:g} x {h_cm:g} cm",
        f"  map      : {stage.source}",
        f"  start    : {stage.start}",
        f"  finish   : {stage.finish}  [cheese]",
        f"  min path : >= {cfg.stage.min_path_steps} computation times",
        "",
        "RAT (no map, no lidar - eye sees only straight ahead)",
        f"  size     : {rat.size_text()}  (fits 1 unit)",
        f"  speed    : {cfg.rat.units_per_tick} unit / computation time",
        "  decision : forward / left / right / backward",
        f"  position : {rat.pos} = ({x_cm:g}, {y_cm:g}) cm",
        f"  heading  : {rat.heading} {rat.ARROW[rat.heading]}",
        f"  rounds   : {rat.decisions} = {rat.steps} walks + {rat.bumps} bumps",
        f"  last     : {rat.last_action} ({rat.brain.mode})",
        f"  smell    : {rat.scent_cm:.1f} cm to the cheese",
        "             (straight line through the walls)",
        f"  explored : {rat.explored} cells, {rat.backtracks} walk-back steps",
        "",
        "TIME",
        f"  limit    : {cfg.time.limit_seconds:g} s ({cfg.time.limit_seconds / 60:g} min)",
        f"  used     : {game.elapsed:.2f} s",
        f"  remaining: {remaining:.2f} s",
        "",
        "TRAIL",
        f"  dark = newest steps, light = older",
        f"  hue rotates every {cfg.trail.hue_cycle_steps} steps",
        "",
        f"STATUS: {game.status}",
    ])


# --------------------------------------------------------------------- ascii
class AsciiRenderer(Renderer):
    def __init__(self, config: GameConfig):
        self.config = config
        self._started = False

    def render(self, game) -> None:
        if not self._started:
            self._started = True
            print(game.stage.ascii_art(rat_pos=game.rat.pos))
        every = max(1, self.config.log_every_steps)
        if game.rat.steps % every == 0:
            remaining = max(0.0, self.config.time.limit_seconds - game.elapsed)
            print(f"[step {game.rat.steps:>4}] pos={game.rat.pos} "
                  f"move={game.rat.last_action:<8} "
                  f"smell={game.rat.scent_cm:7.1f} cm | "
                  f"time used={game.elapsed:6.2f}s remaining={remaining:6.2f}s")

    def finish(self, game) -> None:
        print()
        print(game.stage.ascii_art(rat_pos=game.rat.pos, trail=set(game.rat.trail)))
        print()
        print(hud_text(game))


# ------------------------------------------------------------------------ tk
class TkRenderer(Renderer):
    MARGIN = 16
    COLORS = {"READY": "#455a64", "RUNNING": "#1565c0", "PAUSED": "#e65100",
              "WIN": "#2e7d32", "DEAD": "#c62828", "DEMO": "#6a1b9a",
              "CLOSED": "#555555"}

    def __init__(self, config: GameConfig):
        import tkinter as tk
        self.tk = tk
        self.config = config
        self.px = config.cell_px
        self.closed = False
        self._static_done = False
        self._dots: dict = {}          # cell -> canvas oval id
        self._last_seen: dict = {}     # cell -> newest trail index

        self.root = tk.Tk()
        self.root.title("AUT GAME - Rat Maze")
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        w = config.stage.width_units * self.px + 2 * self.MARGIN
        h = config.stage.height_units * self.px + 2 * self.MARGIN
        self.canvas = tk.Canvas(self.root, width=w, height=h,
                                bg="white", highlightthickness=0)
        self.canvas.pack(side="left", fill="both")
        self.side = tk.Frame(self.root)
        self.side.pack(side="right", fill="both", expand=True)
        self.hud = tk.Label(self.side, font=("Consolas", 10), justify="left",
                            anchor="nw", padx=14, pady=12)
        self.hud.pack(side="top", fill="both", expand=True)

    def _on_close(self):
        self.closed = True
        self.root.destroy()

    # ------------------------------------------------------------- helpers
    def _xy(self, cell, dx=0.5, dy=0.5):
        """Pixel center (or offset point) of a cell."""
        return (self.MARGIN + (cell[0] + dx) * self.px,
                self.MARGIN + (cell[1] + dy) * self.px)

    def reset_canvas(self) -> None:
        """Forget everything drawn (new map / new run)."""
        if self.closed:
            return
        self.canvas.delete("all")
        self._static_done = False
        self._dots = {}
        self._last_seen = {}

    def _draw_static(self, stage):
        c, px, m = self.canvas, self.px, self.MARGIN
        # start / finish cells
        for cell, color in ((stage.start, "#c8e6c9"), (stage.finish, "#fff3c4")):
            x0, y0 = self._xy(cell, 0, 0)
            c.create_rectangle(x0, y0, x0 + px, y0 + px, fill=color, outline="")
        sx, sy = self._xy(stage.start)
        c.create_text(sx, sy, text="S", font=("Consolas", max(8, px // 2), "bold"),
                      fill="#2e7d32")
        # cheese at the finish point
        fx, fy = self._xy(stage.finish)
        r = px * 0.34
        c.create_oval(fx - r, fy - r, fx + r, fy + r,
                      fill="#fbc02d", outline="#c49000", width=2)
        for hx, hy, hr in ((-0.12, -0.1, 0.09), (0.14, 0.05, 0.07), (-0.05, 0.16, 0.06)):
            c.create_oval(fx + hx * px - hr * px, fy + hy * px - hr * px,
                          fx + hx * px + hr * px, fy + hy * px + hr * px,
                          fill="#fff3c4", outline="")
        # walls
        for y in range(stage.height):
            for x in range(stage.width):
                x0, y0 = m + x * px, m + y * px
                if not stage.is_open((x, y), (x, y - 1)):
                    c.create_line(x0, y0, x0 + px, y0, width=2, fill="#303030")
                if not stage.is_open((x, y), (x - 1, y)):
                    c.create_line(x0, y0, x0, y0 + px, width=2, fill="#303030")
        w = m + stage.width * px
        h = m + stage.height * px
        c.create_line(m, h, w, h, width=2, fill="#303030")   # bottom border
        c.create_line(w, m, w, h, width=2, fill="#303030")   # right border

    # ------------------------------------------------------------ the rat
    def _draw_rat(self, rat):
        self.canvas.delete("rat")
        cx, cy = self._xy(rat.pos)
        hx, hy = rat.DIRS[rat.heading]
        px = self.px
        tip = (cx + hx * 0.36 * px, cy + hy * 0.36 * px)
        left = (cx - hx * 0.28 * px - hy * 0.24 * px, cy - hy * 0.28 * px + hx * 0.24 * px)
        right = (cx - hx * 0.28 * px + hy * 0.24 * px, cy - hy * 0.28 * px - hx * 0.24 * px)
        tail = (cx - hx * 0.55 * px, cy - hy * 0.55 * px)
        self.canvas.create_line(cx, cy, *tail, fill="#e57373", width=2, tags="rat")
        self.canvas.create_polygon(*tip, *left, *right, fill="#795548",
                                   outline="#3e2723", width=1, tags="rat")

    # ---------------------------------------------------------- the trail
    def _trail_color(self, seen_step: int, now_step: int) -> str:
        """Strong color for fresh steps, fading to light; hue rotates."""
        cfg = self.config.trail
        hue = (seen_step % cfg.hue_cycle_steps) / cfg.hue_cycle_steps
        age = min(1.0, (now_step - seen_step) / max(1, cfg.fade_steps))
        r, g, b = colorsys.hsv_to_rgb(hue, 0.95, 0.80)   # newest: strong + dark
        r, g, b = (v + (0.93 - v) * age for v in (r, g, b))  # oldest: light
        return f"#{int(r * 255):02x}{int(g * 255):02x}{int(b * 255):02x}"

    def _paint_trail(self, rat):
        trail = rat.trail
        now = len(trail) - 1
        window = self.config.trail.fade_steps
        # newest index per cell inside the fade window (walk it backwards)
        fresh: dict = {}
        for i in range(now, max(-1, now - window - 1), -1):
            fresh.setdefault(trail[i], i)
        for cell, seen in fresh.items():
            self._last_seen[cell] = max(seen, self._last_seen.get(cell, -1))
        r = self.px * 0.16
        for cell, seen in self._last_seen.items():
            color = self._trail_color(seen, now)
            dot = self._dots.get(cell)
            if dot is None:
                x, y = self._xy(cell)
                self._dots[cell] = self.canvas.create_oval(
                    x - r, y - r, x + r, y + r, fill=color, outline="")
            else:
                self.canvas.itemconfig(dot, fill=color)

    # ----------------------------------------------------------- interface
    def render(self, game) -> None:
        if self.closed:
            return
        if not self._static_done:
            self._static_done = True
            self._draw_static(game.stage)
        self._paint_trail(game.rat)
        self._draw_rat(game.rat)
        self.hud.config(text=hud_text(game),
                        fg=self.COLORS.get(game.status, "black"))
        try:
            self.root.update()
        except self.tk.TclError:
            self.closed = True

    def show_banner(self, game) -> None:
        if self.closed or game.status not in ("WIN", "DEAD"):
            return
        msg = "RAT GOT THE CHEESE!" if game.status == "WIN" else "RAT IS DEAD (time over)"
        w = self.config.stage.width_units * self.px + 2 * self.MARGIN
        h = self.config.stage.height_units * self.px + 2 * self.MARGIN
        self.canvas.create_rectangle(w * 0.08, h * 0.42, w * 0.92, h * 0.58,
                                     fill="white", outline=self.COLORS[game.status],
                                     width=3, tags="banner")
        self.canvas.create_text(w / 2, h / 2, text=msg,
                                font=("Consolas", 16, "bold"),
                                fill=self.COLORS[game.status], tags="banner")

    def finish(self, game) -> None:
        """Console-flow ending: show the result and wait for the close."""
        if self.closed:
            return
        self.render(game)
        self.show_banner(game)
        self.root.mainloop()


# ------------------------------------------------------------------- tk app
class TkApp:
    """The playable window: TkRenderer + Start/Stop/map buttons."""

    def __init__(self, game):
        if not isinstance(game.renderer, TkRenderer):
            raise TypeError("TkApp needs a game with the tk renderer")
        self.game = game
        self.view: TkRenderer = game.renderer
        tk = self.view.tk

        bar = tk.Frame(self.view.side, padx=10, pady=8)
        bar.pack(side="bottom", fill="x")
        self.message = tk.Label(bar, font=("Consolas", 9), anchor="w",
                                fg="#455a64", text="press Start to run the rat")
        self.message.pack(side="bottom", fill="x", pady=(6, 0))
        self.buttons = {}
        rows = ((("start", self.on_start), ("stop", self.on_stop),
                 ("reset", self.on_reset), ("random map", self.on_random),
                 ("save map", self.on_save), ("load map", self.on_load)),
                (("set start", self.on_set_start), ("set cheese", self.on_set_cheese),
                 ("random points", self.on_random_points)))
        for row_buttons in rows:
            row = tk.Frame(bar)
            row.pack(side="top", fill="x", pady=1)
            for name, cmd in row_buttons:
                b = tk.Button(row, text=name.title(), width=10, command=cmd)
                b.pack(side="left", padx=2)
                self.buttons[name] = b
        self._placing: str | None = None    # "start"/"cheese" while click-placing
        self.view.canvas.bind("<Button-1>", self.on_canvas_click)

    # -------------------------------------------------------------- buttons
    def _say(self, text: str) -> None:
        self.message.config(text=text)

    def on_start(self):
        if self.game.status in ("WIN", "DEAD"):
            self.game.reset_run()          # same map, fresh rat
            self.view.reset_canvas()
        self.game.start()
        self._say("running - the rat learns the maze as it goes")

    def on_stop(self):
        self.game.pause()
        self._say(f"paused at {self.game.elapsed:.2f} s (clock stopped)")

    def on_reset(self):
        self.game.reset_run()          # SAME maze - only the path starts over
        self.view.reset_canvas()
        self._say("path reset on the same maze - press Start to run again")

    def on_random(self):
        self.game.pause()
        self._say(self.game.new_map(use_saved=False))
        self.view.reset_canvas()

    def _maps_dir(self):
        folder = self.game.stage.map_path("maps/_").parent
        folder.mkdir(parents=True, exist_ok=True)
        return folder

    def _next_file_name(self) -> str:
        """maze.json, maze1.json, maze2.json, ... first name not used yet.

        The dialog lets you type any name you like; this is only the
        suggestion when you don't configure one."""
        if not (self._maps_dir() / "maze.json").exists():
            return "maze.json"
        n = 1
        while (self._maps_dir() / f"maze{n}.json").exists():
            n += 1
        return f"maze{n}.json"

    def on_save(self):
        from tkinter import filedialog
        target = filedialog.asksaveasfilename(
            parent=self.view.root, title="Save map",
            initialdir=self._maps_dir(), initialfile=self._next_file_name(),
            defaultextension=".json", filetypes=[("map files", "*.json")])
        if target:
            self._say(self.game.save_map(target))

    def on_load(self):
        from tkinter import filedialog
        target = filedialog.askopenfilename(
            parent=self.view.root, title="Load map",
            initialdir=self._maps_dir(), filetypes=[("map files", "*.json")])
        if not target:
            return
        self.game.pause()
        self._say(self.game.load_map_file(target))
        self.view.reset_canvas()

    # ------------------------------------------------- start/cheese tiles
    def _arm_placing(self, what: str):
        self.game.pause()
        self._placing = what
        self.view.canvas.config(cursor="crosshair")
        self._say(f"click a tile on the maze to place the {what}")

    def on_set_start(self):
        self._arm_placing("start")

    def on_set_cheese(self):
        self._arm_placing("cheese")

    def on_random_points(self):
        self.game.pause()
        self._say(self.game.random_points())
        self.view.reset_canvas()

    def on_canvas_click(self, event):
        if self._placing is None:
            return
        px, m = self.view.px, self.view.MARGIN
        cell = ((event.x - m) // px, (event.y - m) // px)
        stage = self.game.stage
        if not stage.in_bounds(cell):
            self._say("that click is outside the field - try again")
            return
        what, self._placing = self._placing, None
        self.view.canvas.config(cursor="")
        if what == "start":
            self._say(self.game.set_start(cell))
        else:
            self._say(self.game.set_cheese(cell))
        self.view.reset_canvas()

    def _refresh_buttons(self):
        state = self.game.status
        self.buttons["start"].config(
            state="disabled" if state == "RUNNING" else "normal")
        self.buttons["stop"].config(
            state="normal" if state == "RUNNING" else "disabled")

    # ----------------------------------------------------------------- loop
    def _loop(self):
        if self.view.closed:
            return
        g = self.game
        was_running = g.status == "RUNNING"
        g.tick()
        if was_running and g.status in ("WIN", "DEAD"):
            self.view.render(g)
            self.view.show_banner(g)
            g.print_summary(g.report())
            self._say("WIN - press Start to run again" if g.status == "WIN"
                      else "DEAD - press Start to try again")
        else:
            self.view.render(g)
        self._refresh_buttons()
        delay = max(1, int(self.game.config.time.tick_seconds * 1000))
        self.view.root.after(delay, self._loop)

    def run(self):
        self.game.print_header()
        print("controls: Start / Stop / Random Map / Save Map / Load Map "
              "(buttons in the window)")
        self.view.render(self.game)
        self._refresh_buttons()
        self.view.root.after(50, self._loop)
        self.view.root.mainloop()


# ------------------------------------------------------------------- factory
def make_renderer(config: GameConfig) -> Renderer:
    if config.renderer == "tk":
        try:
            return TkRenderer(config)
        except Exception as exc:  # no display / no tkinter -> keep playing
            print(f"[graphics] tk window unavailable ({exc}) -> ascii mode")
    return AsciiRenderer(config)


# ---------------------------------------------------------------- standalone
class _Snapshot:
    """A frozen game-like object so this file can run separately."""

    def __init__(self, config: GameConfig):
        from rat import Rat
        from stage import Stage
        self.config = config
        self.stage = Stage(config.stage)
        self.rat = Rat(config.rat, config.stage.unit_cm)
        self.rat.place(self.stage.start)
        self.rat.sense(self.stage, self.stage.finish)
        self.elapsed = 0.0
        self.status = "DEMO"


if __name__ == "__main__":
    cfg = GameConfig()
    if "--ascii" in sys.argv:
        cfg.renderer = "ascii"
    snap = _Snapshot(cfg)
    renderer = make_renderer(cfg)
    renderer.render(snap)
    renderer.finish(snap)
    print("graphics demo done (close the window to exit)"
          if isinstance(renderer, TkRenderer) else "graphics demo done")
