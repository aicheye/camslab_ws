"""Fiducial surveying study: python -m camslab_standalone.fiducial [--runs N] [--no-plot]."""

import argparse

from .figures import TITLES
from .study import ERROR_NAMES, monte_carlo, run


def main():
    ap = argparse.ArgumentParser(prog="camslab_standalone.fiducial", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--t-max", type=float, default=35.0)
    ap.add_argument("--dt", type=float, default=0.05, help="camera sample period, s")
    ap.add_argument("--runs", type=int, default=0,
                    help="Monte Carlo runs with the paper's measurement noise; 0 runs "
                         "once without noise")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--no-plot", action="store_true")
    ap.add_argument("--out", metavar="PNG", help="write the figure instead of showing it")
    args = ap.parse_args()

    if args.runs > 0:
        t, e, r = monte_carlo(args.runs, args.seed, t_max=args.t_max, dt=args.dt)
        label = "Simulation study with measurement noise"
    else:
        t, e, r = run(args.t_max, args.dt)
        label = "Simulation study without measurement noise"
    print(f"{label} at t = {t[-1]:.1f} s")
    for name, value, radius in zip(ERROR_NAMES, e[-1], r[-1]):
        print(f"  {name:<22} {value:.3e}   radius {radius:.3e}")

    if args.no_plot:
        return
    from .figures import error_figure

    fig = error_figure(t, e, r, TITLES,
                       "Monte Carlo RMSE" if args.runs > 0 else "Error", "99.73 % radius",
                       f"{label} ({args.runs} runs)" if args.runs > 0 else label)
    if args.out:
        fig.savefig(args.out, dpi=200)
        print(f"figure written to {args.out}")
    else:
        import matplotlib.pyplot as plt
        plt.show()


if __name__ == "__main__":
    main()
