import numpy as np
import pytest

from overtake_core import Circuit, CORNERS, RADII, START, Track, TrafficCar


@pytest.fixture(scope="module")
def circuit():
    return Circuit()


def test_lap_closes_and_has_the_expected_length(circuit):
    track = circuit.track
    # polygon perimeter minus what each 90 degree corner cuts off
    assert track.length == pytest.approx(1380.0 - (2.0 - 0.5 * np.pi) * sum(RADII))
    x, y, psi = track.pose(0.0)
    assert (x, y, psi) == pytest.approx((*START, 0.0))
    x1, y1, psi1 = track.pose(track.length - 1e-9)
    assert (x1, y1) == pytest.approx(START, abs=1e-6)
    assert np.cos(psi1) == pytest.approx(1.0)  # one full clockwise turn later


def test_centreline_is_continuous_and_turns_as_sketched(circuit):
    track = circuit.track
    s = np.linspace(0.0, track.length, 40000, endpoint=False)
    x, y, psi = track.pose(s)
    assert np.hypot(np.diff(x), np.diff(y)).max() == pytest.approx(s[1], rel=1e-3)
    assert np.abs(np.diff(psi)).max() < 0.01
    kappa = track.curvature(s)
    assert kappa.min() == pytest.approx(-1.0 / 15.0)  # tightest right turn
    assert kappa.max() == pytest.approx(1.0 / 15.0)  # the left turn after the 30 m leg
    assert psi[-1] - psi[0] == pytest.approx(-2.0 * np.pi, abs=0.01)  # clockwise


def test_project_inverts_pose(circuit):
    track = circuit.track
    rng = np.random.default_rng(0)
    for s, e_y in zip(rng.uniform(0.0, track.length, 300), rng.uniform(-5.0, 5.0, 300)):
        x, y, _ = track.pose(s, e_y)
        s_back, e_back = track.project(x, y)
        assert s_back == pytest.approx(s, abs=1e-6)
        assert e_back == pytest.approx(e_y, abs=1e-6)


def test_surface_by_lateral_offset(circuit):
    assert circuit.friction(0.0) == 0.9
    assert circuit.friction(-3.4) == 0.9
    assert circuit.friction(3.6) == 0.3
    assert circuit.friction(-5.2) == 0.3
    assert np.isnan(circuit.friction(5.3))


def test_traffic_holds_its_speed_and_lane_through_corners(circuit):
    car = TrafficCar(at=(280.0, 0.0))  # just before the tight corners
    t = np.arange(0.0, 20.0, 0.01)
    poses = np.array([circuit.traffic_pose(car, tk) for tk in t])
    speed = np.hypot(np.diff(poses[:, 0]), np.diff(poses[:, 1])) / 0.01
    assert speed == pytest.approx(50 / 3.6, rel=1e-3)
    assert poses[:, 3].max() > 400.0  # it went through them
    for x, y, _, _ in poses[::100]:
        assert circuit.track.project(x, y)[1] == pytest.approx(-1.75, abs=1e-6)


def test_radii_must_fit():
    with pytest.raises(ValueError):
        Track(CORNERS, (40.0, 20.0, 20.0, 15.0, 25.0, 25.0, 30.0, 30.0), START)
