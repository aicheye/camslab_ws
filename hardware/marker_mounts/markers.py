"""Printable ArUco markers for the stands: one marker per page, drawn as vector squares
at exact size, inside the 200 mm cut outline of the plate face with the chevron that marks
the top. Print at 100 % (no "fit to page") and check one side with a ruler.

    pip install opencv-contrib-python matplotlib
    python3 markers.py                          # ids 0-3, two copies each, Letter
    python3 markers.py --ids 0 1 2 --copies 3 --page a4 --out markers.pdf
"""
import argparse

import cv2
import matplotlib
matplotlib.use("Agg")
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.patches import Polygon, Rectangle
import matplotlib.pyplot as plt

DICTIONARY = "DICT_4X4_50"
MARKER = 140.0                  # printed side [mm], marker_size of config/survey.yaml
PLATE = 200.0                   # plate face and cut square [mm], PLATE of mounts.py
PAGES = {"letter": (215.9, 279.4), "a4": (210.0, 297.0)}
MM = 1 / 25.4


def bits(dictionary: str, marker_id: int):
    """The module matrix of a marker, border included: 1 is black."""
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary))
    n = d.markerSize + 2
    image = cv2.aruco.generateImageMarker(d, marker_id, n, borderBits=1)
    return (image == 0).astype(int)


def page(pdf, dictionary: str, marker_id: int, copy: int, copies: int, size):
    width, height = size
    fig = plt.figure(figsize=(width * MM, height * MM))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, width)
    ax.set_ylim(height, 0)
    ax.set_aspect("equal")
    ax.axis("off")

    cx, cy = width / 2, PLATE / 2 + 12.0
    # cut outline of the plate face
    ax.add_patch(Rectangle((cx - PLATE / 2, cy - PLATE / 2), PLATE, PLATE, fill=False,
                           lw=0.3, ls=(0, (4, 3)), ec="0.4"))
    # the marker, one black square per module
    m = bits(dictionary, marker_id)
    n = m.shape[0]
    cell = MARKER / n
    x0, y0 = cx - MARKER / 2, cy - MARKER / 2
    for r in range(n):
        for c in range(n):
            if m[r, c]:
                ax.add_patch(Rectangle((x0 + c * cell, y0 + r * cell), cell, cell,
                                       fc="black", ec="black", lw=0))
    # chevron toward the top of the marker, as engraved on the plate (mounts.py)
    ax.add_patch(Polygon([(cx - 5, cy - 76), (cx + 5, cy - 76), (cx, cy - 84)],
                         closed=True, fc="black", ec="none"))
    # label and a scale check, outside the cut square
    ty = cy + PLATE / 2 + 10
    ax.text(cx, ty, f"ArUco {dictionary}   id {marker_id}   side {MARKER:.0f} mm"
            f"   copy {copy} of {copies}", ha="center", va="center", fontsize=9)
    ax.plot([cx - 50, cx + 50], [ty + 10, ty + 10], color="black", lw=0.5)
    for x in (cx - 50, cx + 50):
        ax.plot([x, x], [ty + 8, ty + 12], color="black", lw=0.5)
    ax.text(cx, ty + 17, "100 mm when printed at 100 %. Cut on the dashed line; "
            "the chevron goes toward the chevron on the plate.",
            ha="center", va="center", fontsize=7)
    pdf.savefig(fig)
    plt.close(fig)


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--ids", type=int, nargs="+", default=[0, 1, 2, 3])
    p.add_argument("--copies", type=int, default=2)
    p.add_argument("--dictionary", default=DICTIONARY)
    p.add_argument("--page", choices=PAGES, default="letter")
    p.add_argument("--out", default="markers.pdf")
    a = p.parse_args()
    with PdfPages(a.out) as pdf:
        for marker_id in a.ids:
            for copy in range(1, a.copies + 1):
                page(pdf, a.dictionary, marker_id, copy, a.copies, PAGES[a.page])
    print(f"{a.out}: {len(a.ids) * a.copies} pages, ids {a.ids}, {a.copies} copies each, "
          f"{a.page}, marker {MARKER:.0f} mm in a {PLATE:.0f} mm square")


if __name__ == "__main__":
    main()
