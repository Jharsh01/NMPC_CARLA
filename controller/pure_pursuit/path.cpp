#include "controller/pure_pursuit/path.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ov {

const double QUINTIC_PEAK = 10.0 / std::sqrt(3.0);

double quintic(double s) {
    s = std::clamp(s, 0.0, 1.0);
    return s * s * s * (10.0 - 15.0 * s + 6.0 * s * s);
}

double quintic_inverse(double y) {
    double lo = 0.0, hi = 1.0;
    for (int i = 0; i < 60; ++i) {
        const double mid = 0.5 * (lo + hi);
        if (quintic(mid) < y)
            lo = mid;
        else
            hi = mid;
    }
    return 0.5 * (lo + hi);
}

double OvertakePath::y(double X) const {
    return width * (quintic((X - x_out) / length_out) - quintic((X - x_back) / length_back));
}

OvertakePath plan_overtake(const OvertakeScenario& sc, const State& x, double a_lat, double gap_back) {
    const double v = x[IUX], w = sc.lane_width;
    const double closing = v - sc.v_lead;
    if (closing <= 0.0) throw std::invalid_argument("ego is not faster than the lead car");

    const double length_comfort = v * std::sqrt(QUINTIC_PEAK * w / a_lat);
    // distance the ego covers before its front reaches the lead's rear
    const double x_contact = v * sc.clearance(0.0, x) / closing;
    const double s_clear = quintic_inverse((sc.car_width + sc.clearance_min) / w);
    const double length_out = std::min(length_comfort, x_contact / s_clear);
    const double a_lat_out = QUINTIC_PEAK * w * v * v / (length_out * length_out);

    const double t_back = (gap_back - sc.gap_ahead(0.0, x)) / closing;
    const double x_back = std::max(x[IX] + v * t_back, x[IX] + length_out);
    return {w, x[IX], length_out, x_back, length_comfort, a_lat_out, a_lat_out <= sc.mu * 9.81};
}

}  // namespace ov
