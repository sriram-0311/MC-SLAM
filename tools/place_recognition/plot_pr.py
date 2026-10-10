#!/usr/bin/env python3
"""Precision-recall curves (loop closure and relocalization) from evaluate.py --json output.

    plot_pr.py results.json out.png "method=Label" ...
"""
import json
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

SURFACE, INK, INK2, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e6e5e1"
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"]  # categorical slots 1-4, fixed order


def main() -> None:
    report = json.load(open(sys.argv[1]))["report"]
    methods = [a.split("=", 1) for a in sys.argv[3:]]
    fig, axes = plt.subplots(1, 2, figsize=(10, 4.3), facecolor=SURFACE, sharey=True)
    for ax, task, title in zip(axes, ["loop", "reloc"],
                               ["Loop closure (same sequence, >= 15 s back)",
                                "Relocalization (other sequence, same room)"]):
        ax.set_facecolor(SURFACE)
        for (key, label), color in zip(methods, SERIES):
            r = report[key][task]
            precision, recall = r["pr"]
            ax.plot(recall, precision, color=color, lw=2, label=f"{label}   AP {r['AP']:.2f}")
        ax.set_title(title, color=INK, fontsize=10, loc="left")
        ax.set_xlabel("recall", color=INK2)
        ax.set_xlim(0, 1)
        ax.set_ylim(0, 1.02)
        ax.grid(color=GRID, lw=0.8)
        ax.tick_params(colors=INK2, labelsize=8)
        for s in ax.spines.values():
            s.set_visible(False)
        ax.legend(frameon=False, fontsize=8.5, labelcolor=INK, loc="lower left")
    axes[0].set_ylabel("precision", color=INK2)
    fig.suptitle("EuRoC place recognition: top-1 match accepted above a score threshold",
                 color=INK, fontsize=11, x=0.01, ha="left")
    fig.tight_layout()
    fig.savefig(sys.argv[2], dpi=150, facecolor=SURFACE)


if __name__ == "__main__":
    main()
