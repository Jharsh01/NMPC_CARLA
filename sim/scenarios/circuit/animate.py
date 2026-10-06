"""Drawing of the circuit and animation of a lap (matplotlib)."""

import time
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Polygon

from overtake_core import CORNERS, IDELTA, IUX, IX, IY
from sim.video import save_animation

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
    `sim.scenarios.circuit.run.run_lap`. Left: the whole circuit with the cars as markers. Right:
    one view per run that follows its car, to scale, `view` metres to each
    side, and below them speed against distance, with the speed the
    centreline's curvature allows at the road's friction limit. The runs share
    the traffic but do not see each other. A run that has ended stays where it
    stopped. `speed` is the playback rate relative to real time; in a window
    frames are dropped as needed to hold it. With `save` (an .mp4 or .gif path) the
    animation is written to file.
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

    def clock():
        """Frame indices that follow the wall clock, so slow drawing drops frames instead of slowing playback."""
        start = time.perf_counter()
        while True:
            k = int((time.perf_counter() - start) * speed / dt)
            yield min(k, len(t) - 1)
            if k >= len(t) - 1:
                return

    anim = FuncAnimation(
        fig,
        draw,
        frames=frames if save else clock,
        interval=1000.0 / FPS,
        blit=False,
        repeat=False,
        cache_frame_data=False,
    )
    if save:
        save_animation(anim, save, FPS, dpi)
        plt.close(fig)
    else:
        plt.show()
    return anim


def show_circuit(circuit, animate=False, speed=20.0, save=None):
    """Draw the circuit with its lanes, grass strips and traffic, close up on its tight corners.

    With `animate` the traffic circulates at `speed` times real time. With
    `save` the picture (.png) or the animation (.mp4, .gif) is written to file
    instead of opening a window.
    """
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
    if not animate:
        if save:
            Path(save).parent.mkdir(parents=True, exist_ok=True)
            fig.savefig(save, dpi=130)
            plt.close(fig)
        else:
            plt.show()
        return
    lap = track.length / circuit.traffic[0].speed
    anim = FuncAnimation(fig, draw, frames=np.arange(0.0, lap, speed / FPS), interval=1000.0 / FPS, blit=False)
    if save:
        save_animation(anim, save, FPS)
        plt.close(fig)
    else:
        plt.show()
