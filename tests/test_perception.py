import numpy as np
import pytest

from overtake_core import BicyclePlant, Circuit, EGO_SIGMA, Perception, run_lap, TrackPurePursuit, VehicleParams


def test_default_reports_every_car_exactly():
    c = Circuit()
    seen = Perception(c).observe(3.0, 0.0)
    assert len(seen) == len(c.traffic)
    for car, d in zip(c.traffic, seen):
        assert d.s == pytest.approx(float(c.traffic_s(car, 3.0)))
        assert d.e_y == c.lane_offset(car.lane)
        assert d.speed == car.speed


def test_range_limits_what_is_reported():
    c = Circuit()
    # at t = 0 the cars are 60 m and 80 m past the start line, the next one 316 m
    assert len(Perception(c, range_ahead=70.0, range_behind=20.0).observe(0.0, 0.0)) == 1
    assert len(Perception(c, range_ahead=90.0, range_behind=20.0).observe(0.0, 0.0)) == 2
    # from 70 m the slow car is 10 m behind: seen with 20 m of rear range, not with 5 m
    assert len(Perception(c, range_ahead=5.0, range_behind=20.0).observe(0.0, 70.0)) == 1
    assert len(Perception(c, range_ahead=5.0, range_behind=5.0).observe(0.0, 70.0)) == 0
    # range is measured round the lap: just before the line, the cars just after it are ahead
    assert len(Perception(c, range_ahead=90.0, range_behind=20.0).observe(0.0, c.track.length - 5.0)) == 2


def test_noise_has_the_requested_spread_and_is_repeatable():
    c = Circuit()
    p = Perception(c, range_ahead=70.0, range_behind=0.0, sigma_s=0.5, sigma_e=0.2, sigma_v=0.5, seed=1)
    samples = np.array([[d.s, d.e_y, d.speed] for _ in range(4000) for d in p.observe(0.0, 0.0)])
    assert samples.mean(axis=0) == pytest.approx([60.0, -1.75, 30 / 3.6], abs=0.03)
    assert samples.std(axis=0) == pytest.approx([0.5, 0.2, 0.5], rel=0.05)
    again = Perception(c, range_ahead=70.0, range_behind=0.0, sigma_s=0.5, sigma_e=0.2, sigma_v=0.5, seed=1)
    assert again.observe(0.0, 0.0)[0].s == pytest.approx(samples[0, 0])


def test_ego_state_is_exact_by_default_and_noisy_on_request():
    c = Circuit()
    x = c.initial_state()
    assert np.array_equal(Perception(c).ego(x), x)
    sigma = np.array(EGO_SIGMA)
    p = Perception(c, sigma_ego=sigma, seed=1)
    samples = np.array([p.ego(x) for _ in range(4000)])
    assert samples.mean(axis=0) == pytest.approx(x, abs=4.0 * sigma.max() / np.sqrt(4000))
    assert samples.std(axis=0) == pytest.approx(sigma, rel=0.05)
    # the ego errors have their own stream: the traffic errors are the same with and without them
    noisy = dict(range_ahead=70.0, range_behind=0.0, sigma_s=0.5, seed=1)
    with_ego = Perception(c, sigma_ego=sigma, **noisy)
    with_ego.ego(x)
    assert with_ego.observe(0.0, 0.0)[0].s == Perception(c, **noisy).observe(0.0, 0.0)[0].s


def test_road_is_known_only_within_range():
    c = Circuit()
    # the first corner starts 200 m past the start line (15 m radius, to the right)
    corner = c.track.seg_s[1]
    assert c.track.curvature(corner + 1.0) == pytest.approx(-1 / 15)
    p = Perception(c, road_range=20.0)
    ahead = np.array([10.0, 20.0, 21.0, 40.0])
    assert list(p.road_known(100.0, 100.0 + ahead)) == [True, True, False, False]
    assert np.all(p.curvature(corner - 25.0, corner - 25.0 + ahead) == 0.0)  # corner not in view yet
    assert np.all(p.curvature(corner - 15.0, corner - 15.0 + ahead)[1:] == pytest.approx(-1 / 15))
    # the road beyond the range is taken to keep the curvature at its end: here still the first corner
    assert p.curvature(corner - 5.0, corner + 60.0) == pytest.approx(-1 / 15)
    assert Perception(c).curvature(corner - 25.0, corner + 1.0) == pytest.approx(-1 / 15)


@pytest.mark.slow
def test_pure_pursuit_slows_to_what_the_known_road_allows():
    c = Circuit()
    params = VehicleParams()
    limited = TrackPurePursuit(c, params, perception=Perception(c, road_range=20.0))
    full = TrackPurePursuit(c, params)
    s = np.arange(0.0, c.track.length, 5.0)
    v_limited, v_full = limited.speed_reference(s)[0], full.speed_reference(s)[0]
    assert v_limited.min() == pytest.approx(limited.v_limit.min())
    # on a straight: the tightest corner's speed plus what 20 m of braking sheds
    cruise = np.sqrt(limited.v_limit.min() ** 2 + 2.0 * limited.a_brake * 20.0)
    assert v_limited.max() == pytest.approx(cruise, rel=0.01)
    assert cruise < c.v_max
    step = c.track.length / len(limited.v_limit)
    assert np.all(v_limited <= limited.v_limit[(s / step).astype(int)] + 1e-9)
    assert v_full.max() == pytest.approx(c.v_max)

    log, result = run_lap(c, limited, BicyclePlant(params))
    assert result.completed and not result.collided
    assert result.max_speed < cruise + 0.5
    assert result.time_on_grass == 0.0
