"""One-lap circuit: two lanes of asphalt, a grass strip on each side, slower traffic.

The centreline is the lane divider. The circuit is driven clockwise, so the
right lane (e_y < 0) is on the inside of most corners. Traffic cars have no
dynamics: each holds a lane centre at constant speed along that lane.
"""

from dataclasses import dataclass, field
from functools import lru_cache

import numpy as np

from ..model.bicycle import IPSI, IUX, IX, IY, NX
from .overtake import _polygon_distance
from .track import Track, _wrap

# Centreline corners [m], clockwise from the top-left one, and the radius at each.
# The outline follows the sketch: 300 m top straight; on the right side 30 m
# down, 60 m out to the right, 100 m down, 60 m back to the left and 200 m
# down; 300 m along the bottom and 330 m up the left side.
CORNERS = (
    (0.0, 0.0),
    (300.0, 0.0),
    (300.0, -30.0),
    (360.0, -30.0),
    (360.0, -130.0),
    (300.0, -130.0),
    (300.0, -330.0),
    (0.0, -330.0),
)
RADII = (40.0, 15.0, 15.0, 25.0, 25.0, 25.0, 30.0, 30.0)  # the 30 m leg allows at most 15 m at its two ends
START = (100.0, 0.0)  # 100 m after the top-left corner


@dataclass(frozen=True)
class TrafficCar:
    """Constant-speed car on a lane centre. `at` is a world point near its position at t = 0."""

    at: tuple
    lane: str = "right"
    speed: float = 50 / 3.6  # [m/s]


@dataclass(frozen=True)
class Circuit:
    track: Track = field(default_factory=lambda: Track(CORNERS, RADII, START))
    lane_width: float = 3.5  # [m]
    grass_width: float = 1.75  # half a lane on each side [m]
    mu_road: float = 0.9  # asphalt
    mu_grass: float = 0.3
    v_max: float = 80 / 3.6  # ego speed limit [m/s]
    car_length: float = 4.79  # CARLA Model 3 bounding box [m]
    car_width: float = 2.16
    cg_to_center: float = 0.45  # ego centre of gravity ahead of its body centre [m]
    traffic: tuple = (
        # slow car 60 m past the start line: every controller catches it on the top straight
        TrafficCar(at=(160.0, 0.0), speed=30 / 3.6),
        TrafficCar(at=(180.0, 0.0)),  # top straight
        TrafficCar(at=(360.0, -80.0)),  # 100 m straight on the right side
        TrafficCar(at=(300.0, -220.0)),  # 200 m straight
        TrafficCar(at=(160.0, -330.0)),  # bottom straight
        TrafficCar(at=(0.0, -180.0)),  # left side
    )

    @property
    def road_half_width(self):
        return self.lane_width

    @property
    def half_width(self):
        """Centreline to the outer edge of the grass [m]."""
        return self.lane_width + self.grass_width

    def lane_offset(self, lane):
        """e_y of a lane centre [m]."""
        return {"left": 0.5, "right": -0.5}[lane] * self.lane_width

    def friction(self, e_y):
        """Friction coefficient at lateral offset `e_y`; nan beyond the grass."""
        e = np.abs(e_y)
        return np.where(e <= self.road_half_width, self.mu_road, np.where(e <= self.half_width, self.mu_grass, np.nan))

    def traffic_s(self, car, t):
        """Arc length s of a traffic car at time(s) `t`, in 0..lap length."""
        e_y = self.lane_offset(car.lane)
        return self.track.offset_to_s(_start_distance(self.track, car, e_y) + car.speed * np.asarray(t), e_y)

    def traffic_pose(self, car, t):
        """World x, y and heading of a traffic car's body centre at time `t`, and its s."""
        s = self.traffic_s(car, t)
        return (*self.track.pose(s, self.lane_offset(car.lane)), s)

    def initial_state(self, speed=50 / 3.6, lane="right"):
        """Ego state on the start line, on a lane centre, at `speed`."""
        x = np.zeros(NX)
        x[IX], x[IY], x[IPSI] = self.track.pose(0.0, self.lane_offset(lane))
        x[IUX] = speed
        return x

    def track_state(self, x):
        """Ego (s, e_y, e_psi): arc length, lateral offset and heading relative to the track."""
        s, e_y = self.track.project(x[IX], x[IY])
        return s, e_y, _wrap(x[IPSI] - self.track.pose(s)[2])

    def ego_corners(self, x):
        """Corners of the ego body, shape (4, 2)."""
        c, s = np.cos(x[IPSI]), np.sin(x[IPSI])
        return self.body_corners(x[IX] - self.cg_to_center * c, x[IY] - self.cg_to_center * s, x[IPSI])

    def clearance(self, t, x):
        """Distance from the ego body to the nearest traffic car body [m], 0 if they overlap."""
        ego = self.ego_corners(x)
        best = np.inf
        for car in self.traffic:
            cx, cy, psi, _ = self.traffic_pose(car, t)
            gap = np.hypot(cx - x[IX], cy - x[IY]) - self.car_length
            if gap < best:  # only then can the bodies be closer than the best so far
                best = min(best, _polygon_distance(ego, self.body_corners(cx, cy, psi)))
        return best

    def road_margin(self, x):
        """Distance from the ego body to the nearer asphalt edge [m], negative if a corner is on the grass."""
        return self.road_half_width - max(abs(self.track.project(*corner)[1]) for corner in self.ego_corners(x))

    def body_corners(self, x, y, psi):
        """Corners of a car body centred on (x, y), shape (4, 2)."""
        hl, hw = 0.5 * self.car_length, 0.5 * self.car_width
        c, s = np.cos(psi), np.sin(psi)
        local = np.array([[hl, hw], [-hl, hw], [-hl, -hw], [hl, -hw]])
        return np.array([x, y]) + local @ np.array([[c, s], [-s, c]])


@lru_cache(maxsize=None)
def _start_distance(track, car, e_y):
    """Distance along its lane from the start line to where a traffic car is at t = 0."""
    return float(track.offset_length(track.project(*car.at)[0], e_y))
