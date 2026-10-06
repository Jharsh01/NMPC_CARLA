#include "controller/pure_pursuit/speed_profile.hpp"

#include <algorithm>
#include <cmath>

namespace ov {

CorneringLimit cornering_limit(const Track& track, const std::vector<double>& offsets, double a, double v_max,
                               double ds) {
    const int n = static_cast<int>(std::lround(track.length / ds));
    CorneringLimit out;
    for (int i = 0; i < n; ++i) {
        const double s = track.length * i / n;
        const double kappa = track.curvature(s);
        double k_line = 0.0;
        for (double e : offsets)  // curvature of a line offset by e from the centreline is kappa / (1 - kappa e)
            k_line = std::max(k_line, std::fabs(kappa / (1.0 - kappa * e)));
        out.s.push_back(s);
        out.k_line.push_back(k_line);
        out.v.push_back(std::min(v_max, std::sqrt(a / std::max(k_line, 1e-9))));
    }
    return out;
}

// acceleration left along the path while cornering at speed v on curvature k
static double spare(double a, double v, double k) { return std::sqrt(std::max(a * a - (v * v * k) * (v * v * k), 0.0)); }

std::pair<double, double> speed_within_sight(std::vector<double> v, const std::vector<double>& k_line, double step,
                                             double a, double a_brake, double v_beyond) {
    v.back() = std::min(v.back(), v_beyond);
    for (int i = static_cast<int>(v.size()) - 2; i >= 0; --i)
        v[i] = std::min(v[i], std::sqrt(v[i + 1] * v[i + 1] +
                                        2.0 * std::min(spare(a, v[i + 1], k_line[i + 1]), a_brake) * step));
    return {v[0], v[1]};
}

std::pair<std::vector<double>, std::vector<double>> grip_speed_profile(
    const Track& track, const std::vector<double>& offsets, double mu, double v_max, double grip_use,
    double a_accel, double a_brake, double ds, double g) {
    const double a = grip_use * mu * g;
    CorneringLimit limit = cornering_limit(track, offsets, a, v_max, ds);
    std::vector<double>& v = limit.v;
    const std::vector<double>& k = limit.k_line;
    const int n = static_cast<int>(v.size());
    const double step = track.length / n;

    for (int pass = 0; pass < 2; ++pass) {  // twice round, so the passes carry across the start line
        for (int i = n - 1; i >= 0; --i) {  // braking: slow enough here to make the next point
            const int nxt = (i + 1) % n;
            v[i] = std::min(v[i], std::sqrt(v[nxt] * v[nxt] + 2.0 * std::min(spare(a, v[nxt], k[nxt]), a_brake) * step));
        }
        for (int i = 0; i < n; ++i) {  // accelerating: no faster than reachable from the previous point
            const int prev = (i + n - 1) % n;
            v[i] = std::min(v[i],
                            std::sqrt(v[prev] * v[prev] + 2.0 * std::min(spare(a, v[prev], k[prev]), a_accel) * step));
        }
    }
    return {limit.s, v};
}

}  // namespace ov
