"""Matplotlib figures of the fiducial study, in the layout of Fig. 1 of the paper: one
panel per estimated quantity, the error on a log axis, the covariance radius dashed."""

import numpy as np

# Panel titles for the four errors of ERROR_FIELDS, in order.
TITLES = ("Camera orientation error $\\|\\tilde{x}\\|$",
          "Camera position error $\\|\\tilde{t}_X\\|$ [m]",
          "Fiducial orientation error $\\max_j \\|\\tilde{m}_j\\|$",
          "Fiducial position error $\\max_j \\|\\tilde{t}_j\\|$ [m]")


def error_figure(t, e, r, titles=TITLES, error_label="Error", radius_label="99.73 % radius",
                 suptitle="", batch=None):
    """A 2 x 2 figure: per panel the error e[:, k] solid, the radius r[:, k] dashed, both
    on a log axis against time, and with `batch` (4 values) the error of an offline
    batch fit as a dotted horizontal line. Returns the figure."""
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 2, sharex=True, figsize=(9, 5.6), constrained_layout=True)
    for k, (ax, title) in enumerate(zip(axes.ravel(), titles)):
        ax.semilogy(t, e[:, k], color="C0", lw=1.6, label=error_label)
        if r is not None and np.any(np.isfinite(r[:, k])):
            ax.semilogy(t, r[:, k], "--", color="C0", lw=1.2, label=radius_label)
        if batch is not None:
            ax.axhline(batch[k], color="C3", ls=":", lw=1.4, label="Batch least-squares error")
        ax.set_title(title, fontsize=10)
        ax.grid(True, which="major", alpha=0.35)
        ax.grid(True, which="minor", alpha=0.12)
    for ax in axes[-1]:
        ax.set_xlabel("Time [s]")
    axes[0, 0].legend(fontsize=8, loc="upper right")
    if suptitle:
        fig.suptitle(suptitle, fontsize=11)
    return fig
