// What a controller is told: noisy traffic detections, a noisy ego state, the road a limited distance ahead.
#pragma once

#include <limits>
#include <optional>
#include <random>
#include <vector>

#include "sim/scenarios/circuit/scenario.hpp"

namespace ov {

struct Detection {
    double s;  // arc length of the car along the track [m], 0..lap length
    double e_y;  // its lateral offset from the centreline [m]
    double speed;  // its speed along its lane [m/s]
};

// standard deviations of the ego state errors [X, Y, psi, Ux, Uy, r, delta] used by `Perception::limited`:
// m, m, rad, m/s, m/s, rad/s, rad
extern const State EGO_SIGMA;

// Traffic, the ego's own state and the road as known to the ego car.
//
// A car is reported while it is within `range_ahead` in front of the ego or
// `range_behind` behind it, measured along the track. Each report carries
// independent Gaussian errors of standard deviation `sigma_s` [m] along the
// track, `sigma_e` [m] across it and `sigma_v` [m/s] in speed, drawn afresh at
// every call.
//
// The ego state is reported with independent Gaussian errors of standard
// deviation `sigma_ego` (one per state), also drawn afresh at every call.
//
// The road's shape is known up to `road_range` [m] ahead of the ego, measured
// along the centreline. Further on it is reported as continuing with the
// curvature at the end of the known part.
//
// The defaults give exact reports of every car and of the ego state, and the
// whole road.
//
// Not modelled: line of sight (a car round a corner is reported like any other
// within range), missed or false detections, delay, errors that persist from
// one call to the next (bias, drift).
class Perception {
public:
    static constexpr double INF = std::numeric_limits<double>::infinity();

    explicit Perception(const Circuit& circuit, double range_ahead = INF, double range_behind = INF,
                        double sigma_s = 0.0, double sigma_e = 0.0, double sigma_v = 0.0, unsigned seed = 0,
                        std::optional<State> sigma_ego = std::nullopt, double road_range = INF);

    // A plausible sensor set: traffic 60 m ahead and 20 m behind with 0.5 m / 0.2 m / 0.5 m/s errors,
    // ego state errors `EGO_SIGMA`, the road known 40 m ahead.
    static Perception limited(const Circuit& circuit, unsigned seed = 0);

    // Detections at time `t` for an ego at arc length `s_ego`.
    std::vector<Detection> observe(double t, double s_ego);
    // The ego state `x` as measured.
    State ego(const State& x);
    // Whether the road at `s` is known to an ego at `s_ego`; `s` counts on from `s_ego` without wrapping.
    bool road_known(double s_ego, double s) const { return s <= s_ego + road_range; }
    // Centreline curvature at `s` as known to an ego at `s_ego`; `s` counts on from `s_ego` without wrapping.
    double curvature(double s_ego, double s) const;

    Circuit c;
    double range_ahead, range_behind;
    std::array<double, 3> sigma;
    std::optional<State> sigma_ego;
    double road_range;

private:
    std::mt19937_64 rng_, rng_ego_;  // separate streams, so the ego errors leave the traffic errors as they were
    std::normal_distribution<double> normal_, normal_ego_;
};

}  // namespace ov
