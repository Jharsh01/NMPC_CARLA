"""Grip-limited speed profile round a closed track."""

import numpy as np


def grip_speed_profile(track, offsets, mu, v_max, grip_use=0.8, a_accel=3.0, a_brake=np.inf, ds=0.5, g=9.81):
    """Speed [m/s] against arc length for lines at the lateral `offsets` from the centreline.

    At each point the speed is the lowest of `v_max`, the cornering limit
    sqrt(a / |kappa|) of the tightest of the offset lines, what braking can
    still shed before the next corner and what accelerating has reached since
    the last one. a = grip_use * mu * g is the acceleration budget; braking and
    accelerating share it with cornering (friction circle), and are also capped
    at `a_brake` and `a_accel` (what the brakes and the drive can deliver).

    Returns (s, v) on a grid of spacing about `ds` covering one lap.
    """
    n = int(round(track.length / ds))
    s = np.linspace(0.0, track.length, n, endpoint=False)
    step = track.length / n
    kappa = track.curvature(s)
    # curvature of a line offset by e from the centreline is kappa / (1 - kappa e)
    k_line = np.max([np.abs(kappa / (1.0 - kappa * e)) for e in offsets], axis=0)
    a = grip_use * mu * g
    v = np.minimum(v_max, np.sqrt(a / np.maximum(k_line, 1e-9)))

    def spare(i):  # acceleration left along the path while cornering at v[i]
        return np.sqrt(max(a**2 - (v[i] ** 2 * k_line[i]) ** 2, 0.0))

    for _ in range(2):  # twice round, so the passes carry across the start line
        for i in range(n - 1, -1, -1):  # braking: slow enough here to make the next point
            nxt = (i + 1) % n
            v[i] = min(v[i], np.sqrt(v[nxt] ** 2 + 2.0 * min(spare(nxt), a_brake) * step))
        for i in range(n):  # accelerating: no faster than reachable from the previous point
            prev = i - 1
            v[i] = min(v[i], np.sqrt(v[prev] ** 2 + 2.0 * min(spare(prev), a_accel) * step))
    return s, v
