"""Top-down animation of logged overtake runs (matplotlib)."""

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Polygon

from overtake_core import IDELTA, IUX, IX, IY
from sim.video import save_animation

FPS = 25
COLORS = ("tab:blue", "tab:orange", "tab:green", "tab:purple")


def animate(scenario, runs, path=None, speed=1.0, window=(-30.0, 50.0), save=None, name=None):
    """Animate one or more runs of the same scenario on a shared clock.

    `runs` maps a controller label to its `(log, result)` from
    `sim.scenarios.overtake.run.run_overtake`; every run gets its own ego car. A run that
    has ended (collision or overtake completed) is left faded where it stopped.
    The view follows the furthest ego still running; `window` is the visible X
    range around it [m]. `path` (an `OvertakePath`) is drawn as a dashed line
    if given. `speed` is the playback rate relative to real time and `name`
    goes in the title. With `save` (an .mp4 or .gif path) the animation is written to
    file instead of shown.
    """
    sc = scenario
    longest = max((log for log, _ in runs.values()), key=lambda log: len(log.t))
    t = longest.t
    dt = t[1] - t[0]
    frames = np.arange(0, len(t), max(1, int(round(speed / (FPS * dt)))))
    right, left = sc.road_edges

    fig, (ax, ax_y) = plt.subplots(2, 1, figsize=(13, 6.0), gridspec_kw={"height_ratios": [1, 1]})
    fig.suptitle(f"Overtake: {name}" if name else "Overtake")

    # road
    x_start = longest.x[0, IX]
    x_end = max(log.x[-1, IX] for log, _ in runs.values())
    x_road = np.array([x_start + window[0], x_end + window[1] + 200.0])
    ax.fill_between(x_road, right, left, color="0.85", zorder=0)
    ax.plot(x_road, [right, right], "k-", lw=2)
    ax.plot(x_road, [left, left], "k-", lw=2)
    ax.plot(x_road, [0.5 * (right + left)] * 2, "w--", lw=2, dashes=(6, 6))
    if path is not None:
        xp = np.linspace(*x_road, 2000)
        ax.plot(xp, path.y(xp), "--", color="0.35", lw=1, label="pure-pursuit path")
    lead = ax.add_patch(Polygon(sc.lead_corners(0.0), closed=True, fc="0.3", ec="k", label="lead"))

    ax_y.axhline(sc.lane_width, color="0.6", lw=0.8)
    ax_y.axhline(0.0, color="0.6", lw=0.8)
    cars = []
    for (label, (log, result)), color in zip(runs.items(), COLORS):
        trail, = ax.plot([], [], "-", color=color, lw=1, alpha=0.6)
        body = ax.add_patch(Polygon(sc.ego_corners(log.x[0]), closed=True, fc=color, ec="k", label=label))
        cars.append((label, log, result, body, trail))
        ax_y.plot(log.t, log.x[:, IY], color=color, label=label)
        if result.t_complete is not None:
            ax_y.axvline(result.t_complete, color=color, lw=0.8, ls=":")
    ax.set_aspect("equal")
    ax.set_ylim(right - 1.0, left + 1.0)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.legend(loc="lower right", fontsize=8, ncols=len(runs) + 2)

    # lateral position over time
    cursor = ax_y.axvline(0.0, color="k", lw=1)
    ax_y.set_xlabel("t [s]")
    ax_y.set_ylabel("Y [m]")
    ax_y.legend(loc="upper right", fontsize=8)
    status = ax.set_title("", family="monospace", fontsize=9, loc="left")
    width = max(len(label) for label in runs)
    fig.tight_layout()

    def draw(k):
        tk = t[k]
        lead.set_xy(sc.lead_corners(tk))
        lines = [f"t {tk:5.2f} s"]
        x_view = None
        for label, log, result, body, trail in cars:
            j = min(k, len(log.t) - 1)
            xj = log.x[j]
            running = k < len(log.t) - 1
            body.set_xy(sc.ego_corners(xj))
            body.set_alpha(1.0 if running else 0.35)
            trail.set_data(log.x[: j + 1, IX], log.x[: j + 1, IY])
            if running:
                x_view = xj[IX] if x_view is None else max(x_view, xj[IX])
                state = (
                    f"v {xj[IUX] * 3.6:5.1f} km/h   steer {np.degrees(xj[IDELTA]):5.2f} deg   "
                    f"gap ahead {sc.gap_ahead(log.t[j], xj):6.1f} m   clearance {sc.clearance(log.t[j], xj):5.2f} m"
                )
            elif result.passed:
                state = f"PASS, back in lane at {result.t_complete:.1f} s"
            else:
                state = f"FAIL ({', '.join(result.failures)}) at {log.t[-1]:.1f} s"
            lines.append(f"{label:{width}s}  {state}")
        if x_view is None:  # last frame: every run has ended
            x_view = max(log.x[-1, IX] for _, log, *_ in cars)
        ax.set_xlim(x_view + window[0], x_view + window[1])
        cursor.set_xdata([tk, tk])
        status.set_text("\n".join(lines))
        return [lead, cursor, status] + [artist for *_, body, trail in cars for artist in (body, trail)]

    anim = FuncAnimation(fig, draw, frames=frames, interval=1000.0 / FPS, blit=False, repeat=False)
    if save:
        save_animation(anim, save, FPS)
        plt.close(fig)
    else:
        plt.show()
    return anim
