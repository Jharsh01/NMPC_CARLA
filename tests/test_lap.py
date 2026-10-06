import numpy as np
import pytest

from overtake_core import BicyclePlant, Circuit, IUX, run_lap, TrackNMPC, VehicleParams


def test_ego_starts_on_the_right_lane_centre():
    c = Circuit()
    s, e_y, e_psi = c.track_state(c.initial_state())
    assert (s, e_y, e_psi) == pytest.approx((0.0, -1.75, 0.0), abs=1e-9)
    assert c.road_margin(c.initial_state()) == pytest.approx(3.5 - 1.75 - 1.08)


def test_plant_friction_follows_the_surface():
    c = Circuit()
    p = VehicleParams()
    x0 = c.initial_state()
    x0[1] += 1.75 + 4.5  # centre of gravity on the left grass strip
    log, r = run_lap(c, lambda t, x: np.zeros(2), BicyclePlant(p), x0=x0, t_max=0.5)
    assert np.all(log.mu == c.mu_grass)
    assert r.time_on_grass == pytest.approx(0.5, abs=0.02)
    assert r.road_margin < 0.0


@pytest.mark.slow
def test_nmpc_brakes_for_the_tight_corners_and_passes_traffic():
    c = Circuit()
    p = VehicleParams()
    controller = TrackNMPC(c, p)
    log, r = run_lap(c, controller, BicyclePlant(p), t_max=14.0)  # start line to the end of the tight corners
    assert controller.failures == 0
    assert not r.collided and not r.left_track
    assert r.time_on_grass == 0.0 and r.road_margin > 0.0
    assert r.min_clearance > 0.3  # it passes the slow car and the next one on the way
    v = log.x[:, IUX]
    assert v.max() > 0.97 * c.v_max  # reaches the limit on the straight
    in_corner = np.abs(c.track.curvature(log.s)) > 0.06
    assert in_corner.any() and 3.6 * v[in_corner].max() < 60.0  # and slows down for the 15 m corners
    assert log.s[-1] > 240.0


def test_speed_profile_respects_grip_and_brakes():
    from overtake_core import grip_speed_profile

    c = Circuit()
    s, v = grip_speed_profile(c.track, (-1.75, 1.75), mu=0.9, v_max=c.v_max, grip_use=0.8, a_brake=4.0)
    assert v.max() == pytest.approx(c.v_max)
    # slowest where the inside lane of a 15 m corner has a 13.25 m radius
    assert v.min() == pytest.approx(np.sqrt(0.8 * 0.9 * 9.81 * 13.25), rel=1e-3)
    accel = np.diff(v**2) / (2.0 * (s[1] - s[0]))
    assert accel.min() >= -4.0 - 1e-6 and accel.max() <= 3.0 + 1e-6


@pytest.mark.slow
def test_pure_pursuit_follows_its_profile_and_moves_over_for_traffic():
    from overtake_core import TrackPurePursuit

    c = Circuit()
    p = VehicleParams()
    controller = TrackPurePursuit(c, p)
    log, r = run_lap(c, controller, BicyclePlant(p), t_max=16.0)
    assert not r.collided and not r.left_track and r.road_margin > 0.0
    v_ref = controller.speed_reference(log.s)[0]
    assert np.abs(log.x[200:, IUX] - v_ref[200:]).max() < 1.0  # after the first 2 s of speeding up
    # the slow car 60 m ahead at the start: the rule moves the car to the left lane and past it
    assert log.e_y.max() > 1.0
    assert r.min_clearance > 0.3
    slow = c.traffic[0]
    assert log.s[-1] > c.traffic_s(slow, log.t[-1]) + c.car_length
