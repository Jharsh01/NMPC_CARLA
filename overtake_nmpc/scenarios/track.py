"""Closed track built from straights and circular arcs.

The centreline is a polygon whose corners are rounded with a radius each. It
is parametrised by arc length s [m] from the start line, in the direction of
travel. Track coordinates are (s, e_y) with e_y the lateral offset from the
centreline, positive to the left of the direction of travel. Curvature is
positive in left turns.
"""

import numpy as np


class Track:
    def __init__(self, corners, radii, start):
        """`corners` (n, 2) are the polygon vertices in driving order, `radii` the
        corner radius at each, and `start` a point on the edge from corner 0 to
        corner 1 where s = 0."""
        P = np.asarray(corners, dtype=float)
        n = len(P)
        edge = np.roll(P, -1, axis=0) - P  # edge i runs from corner i to corner i + 1
        edge_len = np.linalg.norm(edge, axis=1)
        heading = np.arctan2(edge[:, 1], edge[:, 0])
        turn = _wrap(heading - np.roll(heading, 1))  # heading change at corner i
        tangent = np.asarray(radii, dtype=float) * np.tan(0.5 * np.abs(turn))  # corner to arc end
        straight = edge_len - tangent - np.roll(tangent, -1)
        if np.any(straight < -1e-9):
            raise ValueError("corner radii do not fit on the edges")

        to_start = np.linalg.norm(np.asarray(start, dtype=float) - P[0]) - tangent[0]
        if not 0.0 <= to_start <= straight[0]:
            raise ValueError("start must lie on the straight part of the first edge")

        # (length, curvature) from the start line round to the start line
        pieces = [(straight[0] - to_start, 0.0)]
        for i in list(range(1, n)) + [0]:
            pieces.append((radii[i] * abs(turn[i]), np.sign(turn[i]) / radii[i]))
            pieces.append((straight[i] if i else to_start, 0.0))
        pieces = [(length, kappa) for length, kappa in pieces if length > 1e-9]

        self.seg_length = np.array([length for length, _ in pieces])
        self.seg_kappa = np.array([kappa for _, kappa in pieces])
        self.seg_s = np.concatenate([[0.0], np.cumsum(self.seg_length)])  # start of each piece, then the lap length
        self.length = self.seg_s[-1]

        x, y, psi = float(start[0]), float(start[1]), heading[0]
        self.seg_pose = np.zeros((len(pieces), 3))
        for k, (length, kappa) in enumerate(pieces):
            self.seg_pose[k] = x, y, psi
            x, y, psi = _advance(x, y, psi, kappa, length)
        if np.hypot(x - start[0], y - start[1]) > 1e-6:
            raise ValueError("track does not close")

    def _piece(self, s):
        s = np.mod(s, self.length)
        k = np.clip(np.searchsorted(self.seg_s, s, side="right") - 1, 0, len(self.seg_length) - 1)
        return k, s - self.seg_s[k]

    def curvature(self, s):
        return self.seg_kappa[self._piece(s)[0]]

    def pose(self, s, e_y=0.0):
        """World x, y and track heading at arc length `s`, offset `e_y` to the left."""
        k, ds = self._piece(s)
        x, y, psi = _advance(*self.seg_pose[k].T, self.seg_kappa[k], ds)
        return x - e_y * np.sin(psi), y + e_y * np.cos(psi), psi

    def project(self, x, y):
        """Track coordinates (s, e_y) of the world point (x, y): the nearest centreline point."""
        best = None
        for k in range(len(self.seg_length)):
            x0, y0, psi0 = self.seg_pose[k]
            kappa, length = self.seg_kappa[k], self.seg_length[k]
            if kappa == 0.0:
                ds = (x - x0) * np.cos(psi0) + (y - y0) * np.sin(psi0)
            else:
                # angle swept round the arc's centre, measured in the direction of travel
                cx, cy = x0 - np.sin(psi0) / kappa, y0 + np.cos(psi0) / kappa
                swept = np.sign(kappa) * _wrap(np.arctan2(y - cy, x - cx) - np.arctan2(y0 - cy, x0 - cx))
                ds = swept / abs(kappa)
                if ds < -0.5 * (2.0 * np.pi / abs(kappa) - length):  # nearer the far end, the other way round
                    ds += 2.0 * np.pi / abs(kappa)
            ds = np.clip(ds, 0.0, length)
            qx, qy, psi = _advance(x0, y0, psi0, kappa, ds)
            e_y = -(x - qx) * np.sin(psi) + (y - qy) * np.cos(psi)
            dist = np.hypot(x - qx, y - qy)
            if best is None or dist < best[0]:
                best = dist, self.seg_s[k] + ds, e_y
        return best[1] % self.length, best[2]

    def offset_length(self, s, e_y):
        """Distance travelled from the start line to `s` (0..length) along the line offset by `e_y`."""
        scale = 1.0 - self.seg_kappa * e_y  # an inside line is shorter
        cumulative = np.concatenate([[0.0], np.cumsum(self.seg_length * scale)])
        return np.interp(s, self.seg_s, cumulative)

    def offset_to_s(self, distance, e_y):
        """Inverse of `offset_length`, wrapping round the lap."""
        scale = 1.0 - self.seg_kappa * e_y
        cumulative = np.concatenate([[0.0], np.cumsum(self.seg_length * scale)])
        return np.interp(np.mod(distance, cumulative[-1]), cumulative, self.seg_s)


def _wrap(angle):
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def _advance(x, y, psi, kappa, ds):
    """Pose after travelling `ds` from (x, y, psi) at constant curvature."""
    k = np.where(kappa == 0.0, 1.0, kappa)
    end = psi + kappa * ds
    arc_x, arc_y = (np.sin(end) - np.sin(psi)) / k, -(np.cos(end) - np.cos(psi)) / k
    straight = kappa == 0.0
    return (
        x + np.where(straight, ds * np.cos(psi), arc_x),
        y + np.where(straight, ds * np.sin(psi), arc_y),
        end,
    )
