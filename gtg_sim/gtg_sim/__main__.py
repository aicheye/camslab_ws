"""python -m gtg_sim            serve the web UI
python -m gtg_sim --csv out.csv   run once without the UI and write the samples
"""

import argparse
import csv
import dataclasses
import logging
import math
import random

from gtg_webui.server import SAMPLE_FIELDS, UiServer

from .backend import SimBackend
from .simulate import simulate
from .types import SHAPES, Params, State


def main():
    ap = argparse.ArgumentParser(prog="gtg_sim", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--initial", type=float, nargs=3, metavar=("X", "Y", "THETA_DEG"),
                    help="default: random, x and y in [0, RANGE], any heading")
    ap.add_argument("--random-range", type=float, default=4.0, metavar="RANGE")
    ap.add_argument("--seed", type=int, help="seed for the random initial pose")
    ap.add_argument("--dt", type=float, default=0.01)
    ap.add_argument("--t-max", type=float, default=40.0)
    # One --<name> option per field of Params, e.g. --shape gerono --k-perp 4.
    for field in dataclasses.fields(Params):
        flag = "--" + field.name.replace("_", "-")
        if field.type is str or field.type == "str":
            ap.add_argument(flag, default=field.default, choices=SHAPES)
        else:
            ap.add_argument(flag, type=float, default=field.default)
    ap.add_argument("--csv", metavar="PATH", help="run once, write samples, exit")
    args = ap.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(name)s: %(message)s")
    params = Params(**{f.name: getattr(args, f.name) for f in dataclasses.fields(Params)})
    if args.initial:
        x, y, theta_deg = args.initial
        initial = {"x": x, "y": y, "theta": math.radians(theta_deg)}
    else:
        rng = random.Random(args.seed)
        r = args.random_range
        initial = {"x": round(rng.uniform(0, r), 2), "y": round(rng.uniform(0, r), 2),
                   "theta": rng.uniform(-math.pi, math.pi)}

    if args.csv:
        samples = simulate(State.from_pose(**initial), params, args.dt, args.t_max)
        with open(args.csv, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(SAMPLE_FIELDS)
            writer.writerows(samples)
        last = samples[-1]
        print(f"{len(samples)} samples, ||p* - p|| = {last[7]:.4f} m at t = {last[0]:.2f} s")
        return

    backend = SimBackend(params, initial, args.dt, args.t_max)
    backend.server = UiServer(backend, args.host, args.port)
    backend.server.run()


if __name__ == "__main__":
    main()
