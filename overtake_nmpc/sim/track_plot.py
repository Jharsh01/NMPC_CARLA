"""Drawing of the circuit and animation of a lap (matplotlib)."""

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Polygon

from ..model.bicycle import IDELTA, IUX, IX, IY

GRASS, ROAD, TRAFFIC = "#7fb069", "0.8", "tab:red"
COLORS = ("tab:orange", "tab:blue", "tab:purple")  # one per run
FPS = 25


def band(track, s, inner, outer):
    """Closed outline of the strip between two lateral offsets."""
    xo, yo, _ = track.pose(s, outer)
    xi, yi, _ = track.pose(s, inner)
    return np.column_stack([np.concatenate([xo, xi[::-1]]), np.concatenate([yo, yi[::-1]])])


def draw_track(ax, circuit):
    """Asphalt, grass strips, edge lines, lane divider and the start line."""
    track = circuit.track
    s = np.linspace(0.0, track.length, 8000)
    road, edge = circuit.road_half_width, circuit.half_width
    for inner, outer in ((road, edge), (-edge, -road)):
        ax.add_patch(Polygon(band(track, s, inner, outer), fc=GRASS, ec="none"))
    ax.add_patch(Polygon(band(track, s, -road, road), fc=ROAD, ec="none"))
    for offset in (-road, road):
        x, y, _ = track.pose(s, offset)
        ax.plot(x, y, "k-", lw=0.8)
    x, y, _ = track.pose(s)
    ax.plot(x, y, "w--", lw=0.8, dashes=(6, 6))
    start = np.array([track.pose(0.0, e)[:2] for e in (-road, road)])
    ax.plot(start[:, 0], start[:, 1], "k-", lw=2.5)
    ax.set_aspect("equal")


def animate_lap(circuit, runs, speed=1.0, view=35.0, save=None, dpi=100):
    """Animate one or more laps of the same circuit on a shared clock.

    `runs` maps a controller label to its `(log, result)` from
    `sim.lap.run_lap`. Left: the whole circuit with the cars as markers. Right:
    one view per run that follows its car, to scale, `view` metres to each
    side, and below them speed against distance, with the speed the
    centreline's curvature allows at the road's friction limit. The runs share
    the traffic but do not see each other. A run that has ended stays where it
    stopped. `speed` is the playback rate relative to real time. With `save`
    (a .gif path) the animation is written to file.
    """
    c, track = circuit, circuit.track
    longest = max((log for log, _ in runs.values()), key=lambda log: len(log.t))
    t = longest.t
    dt = t[1] - t[0]
    frames = np.arange(0, len(t), max(1, int(round(speed / (FPS * dt)))))

    fig = plt.figure(figsize=(14, 8.0))
    grid = fig.add_gridspec(len(runs) + 1, 2, width_ratios=[1.15, 1], height_ratios=[1.0] * len(runs) + [0.9])
    ax = fig.add_subplot(grid[:, 0])
    ax_v = fig.add_subplot(grid[-1, 1])
    fig.suptitle(f"Lap of the circuit ({track.length:.0f} m)")

    draw_track(ax, c)
    traffic_dots, = ax.plot([], [], "s", color=TRAFFIC, ms=6, mec="k", label="traffic")
    ax.margins(0.06)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")

    s_line = np.linspace(0.0, track.length, 4000)
    grip = np.sqrt(c.mu_road * 9.81 / np.maximum(np.abs(track.curvature(s_line)), 1e-9))
    ax_v.plot(s_line, 3.6 * np.minimum(grip, c.v_max), "--", color="0.4", lw=1, label="80 km/h or centreline grip")
    for i, v in enumerate(sorted({car.speed for car in c.traffic})):
        ax_v.axhline(3.6 * v, color=TRAFFIC, lw=0.8, label=None if i else "traffic")

    cars = []
    for row, ((label, (log, result)), color) in enumerate(zip(runs.items(), COLORS)):
        dot, = ax.plot([], [], "o", color=color, ms=8, mec="k", label=label)
        cam = fig.add_subplot(grid[row, 1])
        draw_track(cam, c)
        trail, = cam.plot([], [], "-", color=color, lw=1, alpha=0.7)
        bodies = [cam.add_patch(Polygon(np.zeros((4, 2)), fc=TRAFFIC, ec="k")) for _ in c.traffic]
        body = cam.add_patch(Polygon(c.ego_corners(log.x[0]), fc=color, ec="k"))
        cam.set_xticks([])
        cam.set_yticks([])
        status = cam.set_title("", family="monospace", fontsize=9, loc="left")
        ax_v.plot(log.s, 3.6 * log.x[:, IUX], color=color, label=label)
        cursor = ax_v.axvline(0.0, color=color, lw=1)
        cars.append((label, log, result, dot, cam, trail, bodies, body, status, cursor))
    ax.legend(loc="center")
    ax_v.set_xlim(0.0, track.length)
    ax_v.set_ylim(0.0, 3.6 * c.v_max + 12.0)
    ax_v.set_xlabel("distance from the start line [m]")
    ax_v.set_ylabel("speed [km/h]")
    ax_v.legend(loc="lower right", fontsize=7, ncols=4)
    fig.tight_layout()

    def draw(k):
        tk = t[k]
        poses = np.array([c.traffic_pose(car, tk) for car in c.traffic])
        traffic_dots.set_data(poses[:, 0], poses[:, 1])
        artists = [traffic_dots]
        for label, log, result, dot, cam, trail, bodies, body, status, cursor in cars:
            j = min(k, len(log.t) - 1)
            xj = log.x[j]
            dot.set_data([xj[IX]], [xj[IY]])
            for patch, (px, py, psi, _) in zip(bodies, poses):
                patch.set_xy(c.body_corners(px, py, psi))
            body.set_xy(c.ego_corners(xj))
            trail.set_data(log.x[: j + 1, IX], log.x[: j + 1, IY])
            cam.set_xlim(xj[IX] - view, xj[IX] + view)
            cam.set_ylim(xj[IY] - 0.36 * view, xj[IY] + 0.36 * view)
            cursor.set_xdata([log.s[j], log.s[j]])
            if k < len(log.t) - 1:
                state = (
                    f"t {tk:5.1f} s   v {xj[IUX] * 3.6:5.1f} km/h   steer {np.degrees(xj[IDELTA]):5.1f} deg   "
                    f"offset {log.e_y[j]:5.2f} m"
                )
            elif result.completed:
                state = f"lap complete: {result.lap_time:.1f} s"
            else:
                state = "collision" if result.collided else "left the track" if result.left_track else "out of time"
                state += f" at {log.t[-1]:.1f} s"
            status.set_text(f"{label}:  {state}")
            artists += [dot, body, trail, status, cursor, *bodies]
        return artists

    anim = FuncAnimation(fig, draw, frames=frames, interval=1000.0 / FPS, blit=False, repeat=False)
    if save:
        anim.save(save, writer=PillowWriter(fps=FPS), dpi=dpi)
        plt.close(fig)
    else:
        plt.show()
    return anim
