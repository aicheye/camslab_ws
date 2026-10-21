"""Fiducial surveying study: python -m camslab_offline.fiducial [--runs N] [--no-plot]."""

import argparse

from .study import ERROR_NAMES, monte_carlo, run


def main():
    ap = argparse.ArgumentParser(prog="camslab_offline.fiducial", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--t-max", type=float, default=35.0)
    ap.add_argument("--dt", type=float, default=0.05, help="camera sample period, s")
    ap.add_argument("--runs", type=int, default=0,
                    help="Monte Carlo runs with the paper's measurement noise; 0 runs "
                         "once without noise")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--no-plot", action="store_true")
    args = ap.parse_args()

    if args.runs > 0:
        t, e = monte_carlo(args.runs, args.seed, t_max=args.t_max, dt=args.dt)
        label = f"RMSE over {args.runs} noisy runs"
    else:
        t, e = run(args.t_max, args.dt)
        label = "error without noise"
    print(f"{label} at t = {t[-1]:.1f} s")
    for name, value in zip(ERROR_NAMES, e[-1]):
        print(f"  {name:<22} {value:.3e}")

    if args.no_plot:
        return
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 2, sharex=True, figsize=(10, 6))
    for ax, name, column in zip(axes.ravel(), ERROR_NAMES, e.T):
        ax.semilogy(t, column)
        ax.set_ylabel(name)
        ax.grid(True, which="both", alpha=0.3)
    for ax in axes[-1]:
        ax.set_xlabel("t [s]")
    fig.suptitle(label)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
