"""AUT GAME - entry point.

    python main.py                     -> game window with buttons:
                                          Start / Stop / Random map / Save map / Load map
    python main.py --renderer ascii    -> play in the console (runs immediately)
    python main.py --save-map          -> save the starting maze right away
    python main.py --use-map           -> begin on the saved map
    python main.py --seed 7 --tick 0.02 --width 30 --height 30

Everything is plain OOP: build a GameConfig, hand it to Game, call run().
Tweak the defaults in config.py (or pass flags here) to re-configure the
game in the future.
"""
import argparse

from config import GameConfig
from game import Game


def parse_args(argv=None) -> argparse.Namespace:
    cfg = GameConfig()  # defaults for the help text
    p = argparse.ArgumentParser(description="AUT GAME - rat maze")
    p.add_argument("--renderer", choices=("tk", "ascii"), default=cfg.renderer,
                   help="tk = infographic window (default), ascii = console")
    p.add_argument("--seed", type=int, default=cfg.stage.seed,
                   help="maze seed (default: random each run)")
    p.add_argument("--width", type=int, default=cfg.stage.width_units,
                   help="maze width in units (default 30)")
    p.add_argument("--height", type=int, default=cfg.stage.height_units,
                   help="maze height in units (default 30)")
    p.add_argument("--tick", type=float, default=cfg.time.tick_seconds,
                   help="seconds per computation time (default 0.05)")
    p.add_argument("--limit", type=float, default=cfg.time.limit_seconds,
                   help="seconds the rat can run before it dies (default 180)")
    p.add_argument("--use-map", action="store_true",
                   help="play the saved map (default: the random switch is on)")
    p.add_argument("--save-map", action="store_true",
                   help="save this run's map so --use-map can play it again")
    p.add_argument("--map-file", default=cfg.stage.map_file,
                   help="map file for --use-map/--save-map (in the stage folder)")
    p.add_argument("--start", type=int, nargs=2, metavar=("X", "Y"), default=None,
                   help="start tile (default: corner 0 0)")
    p.add_argument("--cheese", type=int, nargs=2, metavar=("X", "Y"), default=None,
                   help="cheese tile (default: the opposite corner)")
    p.add_argument("--random-points", action="store_true",
                   help="roll random start and cheese tiles")
    return p.parse_args(argv)


def build_config(args: argparse.Namespace) -> GameConfig:
    cfg = GameConfig()
    cfg.renderer = args.renderer
    cfg.stage.seed = args.seed
    cfg.stage.width_units = args.width
    cfg.stage.height_units = args.height
    cfg.stage.use_saved_map = args.use_map
    cfg.stage.map_file = args.map_file
    cfg.stage.start_point = tuple(args.start) if args.start else None
    cfg.stage.finish_point = tuple(args.cheese) if args.cheese else None
    cfg.stage.random_points = args.random_points
    cfg.time.tick_seconds = args.tick
    cfg.time.limit_seconds = args.limit
    return cfg


def main(argv=None) -> None:
    args = parse_args(argv)
    from stage import StageValidationError
    try:
        game = Game(build_config(args))
    except StageValidationError as exc:
        print(f"bad start/cheese tiles: {exc}")
        raise SystemExit(1)
    if args.save_map:
        print(f"map saved to {game.stage.save()}")
    from graphics import TkApp, TkRenderer
    if isinstance(game.renderer, TkRenderer):
        TkApp(game).run()   # window with Start/Stop/Random/Save/Load buttons
    else:
        game.run()          # console mode runs immediately


if __name__ == "__main__":
    main()
