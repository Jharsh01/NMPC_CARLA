#include "plant/bicycle.hpp"

#include <algorithm>
#include <limits>

namespace ov {

std::pair<double, double> fx_limits(double Ux, const VehicleParams& p) {
    const double grip = p.mu * p.m * p.g;
    auto axle_limit = [](double num, double den) {
        return den > 0.0 ? num / den : std::numeric_limits<double>::infinity();
    };
    const double fb = p.brake_front, fd = p.drive_front, L = p.L();
    const double brake = std::min({axle_limit(grip * p.b, fb * L - p.mu * p.h),
                                   axle_limit(grip * p.a, (1.0 - fb) * L + p.mu * p.h), p.F_brake_max});
    const double drive = std::min({axle_limit(grip * p.b, fd * L + p.mu * p.h),
                                   axle_limit(grip * p.a, (1.0 - fd) * L - p.mu * p.h), p.F_drive_max,
                                   p.P_max / std::max(Ux, p.Ux_min)});
    return {-brake, drive};
}

}  // namespace ov
