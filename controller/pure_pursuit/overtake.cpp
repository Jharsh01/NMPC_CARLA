#include "controller/pure_pursuit/overtake.hpp"

#include <algorithm>
#include <cmath>

namespace ov {

PurePursuit::PurePursuit(const OvertakeScenario& scenario, const VehicleParams& params, double k_lookahead,
                         double lookahead_min, double tau_steer, double k_speed, double a_lat)
    : sc(scenario),
      p(params),
      k_lookahead(k_lookahead),
      lookahead_min(lookahead_min),
      tau_steer(tau_steer),
      k_speed(k_speed),
      a_lat(a_lat) {}

Input PurePursuit::operator()(double, const State& x) {
    if (!path) {  // plan once, at the trigger
        path = plan_overtake(sc, x, a_lat);
        v_ref = x[IUX];
    }

    const double psi = x[IPSI];
    const Eigen::Vector2d rear(x[IX] - p.b * std::cos(psi), x[IY] - p.b * std::sin(psi));
    const double L_d = std::max(k_lookahead * x[IUX], lookahead_min);
    const Eigen::Vector2d target = lookahead_point(rear, L_d);
    const double alpha = std::atan2(target[1] - rear[1], target[0] - rear[0]) - psi;
    const double delta_cmd = std::clamp(std::atan(2.0 * p.L() * std::sin(alpha) / L_d), -p.delta_max, p.delta_max);

    const double rate = (delta_cmd - x[IDELTA]) / tau_steer;
    const double Fx = k_speed * p.m * (v_ref - x[IUX]);
    return {rate, Fx};
}

Eigen::Vector2d PurePursuit::lookahead_point(const Eigen::Vector2d& rear, double L_d) const {
    // the last sample is L_d ahead along X, so at least L_d away
    constexpr int n = 101;
    double X_prev = rear[0], d_prev = 0.0;
    for (int k = 0; k < n; ++k) {
        const double X = rear[0] + L_d * k / (n - 1);
        const double d = std::hypot(X - rear[0], path->y(X) - rear[1]);
        if (d >= L_d) {
            // already L_d off the path at the first sample: aim at the nearest-ahead point
            const double X_la = k == 0 ? X : X_prev + (L_d - d_prev) / (d - d_prev) * (X - X_prev);
            return {X_la, path->y(X_la)};
        }
        X_prev = X;
        d_prev = d;
    }
    return {rear[0], path->y(rear[0])};  // no sample reached L_d (numpy's argmax of all-False is 0)
}

}  // namespace ov
