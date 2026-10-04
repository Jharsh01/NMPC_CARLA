"""Draw the circuit with its lanes, grass strips and traffic, or animate the traffic.

    python scripts/show_track.py                       # opens a window
    python scripts/show_track.py --save results/track.png
    python scripts/show_track.py --animate             # traffic circulating, 20x real time
    python scripts/show_track.py --animate --save results/track.gif
"""

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Polygon

from overtake_nmpc.scenarios.circuit import CORNERS, Circuit
from overtake_nmpc.sim.track_plot import TRAFFIC, draw_track

FPS = 25


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--save", help="write a .png (or a .gif with --animate) instead of opening a window")
    parser.add_argument("--animate", action="store_true", help="animate the traffic")
    parser.add_argument("--speed", type=float, default=20.0, help="playback rate relative to real time")
    args = parser.parse_args()

    circuit = Circuit()
    track = circuit.track
    fig, (ax, ax_zoom) = plt.subplots(1, 2, figsize=(14, 7.5), gridspec_kw={"width_ratios": [1.5, 1]})
    fig.suptitle(
        f"Circuit: lap {track.length:.0f} m, clockwise   |   2 lanes x {circuit.lane_width} m, mu {circuit.mu_road}"
        f"   |   grass {circuit.grass_width} m each side, mu {circuit.mu_grass}"
        f"   |   ego limit {circuit.v_max * 3.6:.0f} km/h"
    )

    # whole lap; the cars are far too small to see at this scale, so they are drawn as markers
    draw_track(ax, circuit)
    ax.annotate("start / finish", track.pose(0.0)[:2], xytext=(0, 14), textcoords="offset points", ha="center")
    x, y, psi = track.pose(40.0, 12.0)
    ax.annotate("", (x + 40.0, y), (x, y), arrowprops=dict(arrowstyle="->"))
    markers, = ax.plot([], [], "s", color=TRAFFIC, ms=7, mec="k", label="traffic")
    ax.margins(0.06)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.legend(loc="center")

    # the right-hand side with its tight corners, close up
    draw_track(ax_zoom, circuit)
    tight = np.array(CORNERS[1:6])
    ax_zoom.set_xlim(tight[:, 0].min() - 25.0, tight[:, 0].max() + 25.0)
    ax_zoom.set_ylim(tight[:, 1].min() - 25.0, tight[:, 1].max() + 25.0)
    ax_zoom.set_title("Right-hand side, close up", fontsize=10)
    ax_zoom.set_xlabel("x [m]")
    bodies = [ax_zoom.add_patch(Polygon(np.zeros((4, 2)), fc=TRAFFIC, ec="k")) for _ in circuit.traffic]
    clock = ax.set_title("", loc="left", family="monospace", fontsize=9)
    fig.tight_layout()

    def draw(t):
        poses = np.array([circuit.traffic_pose(car, t) for car in circuit.traffic])
        markers.set_data(poses[:, 0], poses[:, 1])
        for body, (x, y, psi, _) in zip(bodies, poses):
            body.set_xy(circuit.body_corners(x, y, psi))
        clock.set_text(f"t {t:6.1f} s")
        return [markers, clock, *bodies]

    draw(0.0)
    if args.save:
        Path(args.save).parent.mkdir(parents=True, exist_ok=True)
    if not args.animate:
        if args.save:
            fig.savefig(args.save, dpi=130)
        else:
            plt.show()
        return
    lap = track.length / circuit.traffic[0].speed
    frames = np.arange(0.0, lap, args.speed / FPS)
    anim = FuncAnimation(fig, draw, frames=frames, interval=1000.0 / FPS, blit=False)
    if args.save:
        anim.save(args.save, writer=PillowWriter(fps=FPS))
    else:
        plt.show()


if __name__ == "__main__":
    main()
