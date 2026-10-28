"""Mass-spring-damper estimation: python -m camslab_standalone.msd [--no-plot]."""

import argparse

import numpy as np

from .online import run
from .plant import MsdParams


def main():
    ap = argparse.ArgumentParser(prog="camslab_standalone.msd", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--t-max", type=float, default=60.0)
    ap.add_argument("--dt", type=float, default=1e-3, help="output sample spacing, s")
    ap.add_argument("--lam", type=float, default=2.0, help="filter pole")
    ap.add_argument("--gamma", type=float, default=200.0, help="gradient gain, gamma * I")
    ap.add_argument("--no-plot", action="store_true")
    args = ap.parse_args()

    params = MsdParams()
    t, theta = run(params, args.t_max, args.dt, args.lam, args.gamma * np.eye(3))

    names = ["m", "b", "k"]
    print("true      " + "  ".join(f"{s}={v:7.4f}" for s, v in zip(names, params.theta())))
    print("estimate  " + "  ".join(f"{s}={v:7.4f}" for s, v in zip(names, theta[-1])))

    if args.no_plot:
        return
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(8, 7))
    for i, ax in enumerate(axes):
        ax.plot(t, theta[:, i], label="estimate")
        ax.axhline(params.theta()[i], color="k", ls="--", lw=1, label="true")
        ax.set_ylabel(names[i])
    axes[0].legend()
    axes[-1].set_xlabel("t [s]")
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
