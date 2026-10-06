// Pure-pursuit baseline: track the planned overtake path, hold the entry speed.
#pragma once

#include <optional>

#include "controller/interface.hpp"
#include "controller/pure_pursuit/path.hpp"

namespace ov {

// Steering: the lookahead point is on the path at distance
// L_d = max(k_lookahead * U_x, lookahead_min) from the rear axle, and the
// steering angle that puts the rear axle on an arc through it is
// delta = atan(2 L sin(alpha) / L_d). The plant takes a steering rate, so the
// angle is reached with a first-order command of time constant `tau_steer`.
// Speed: proportional control of F_x toward the entry speed.
class PurePursuit : public Controller {
public:
    PurePursuit(const OvertakeScenario& scenario, const VehicleParams& params, double k_lookahead = 1.4,
                double lookahead_min = 5.0, double tau_steer = 0.05, double k_speed = 1.0, double a_lat = 2.0);

    Input operator()(double t, const State& x) override;
    // First point ahead on the path at distance L_d from `rear`.
    Eigen::Vector2d lookahead_point(const Eigen::Vector2d& rear, double L_d) const;

    OvertakeScenario sc;
    VehicleParams p;
    double k_lookahead, lookahead_min, tau_steer, k_speed, a_lat;
    std::optional<OvertakePath> path;  // planned at the first call
    double v_ref = 0.0;
};

}  // namespace ov
