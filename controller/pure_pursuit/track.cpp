#include "controller/pure_pursuit/track.hpp"

#include <algorithm>
#include <cmath>

#include "controller/pure_pursuit/speed_profile.hpp"

namespace ov {

// Periodic linear interpolation of the samples (xp, fp) of period `period` (as numpy.interp with `period`).
static double interp_periodic(double x, const std::vector<double>& xp, const std::vector<double>& fp,
                              double period) {
    x = pymod(x, period);
    const size_t n = xp.size();
    const size_t i = std::upper_bound(xp.begin(), xp.end(), x) - xp.begin();  // xp[i - 1] <= x < xp[i]
    const size_t lo = (i + n - 1) % n, hi = i % n;
    const double x_lo = i == 0 ? xp[lo] - period : xp[lo];
    const double x_hi = i == n ? xp[hi] + period : xp[hi];
    return fp[lo] + (x - x_lo) / (x_hi - x_lo) * (fp[hi] - fp[lo]);
}

TrackPurePursuit::TrackPurePursuit(const Circuit& circuit, const VehicleParams& params, const Options& options,
                                   std::optional<Perception> perception_)
    : c(circuit),
      p(params),
      opt(options),
      perception(perception_ ? std::move(*perception_) : Perception(circuit)),
      e_home(circuit.lane_offset(options.lane)),
      e_pass(-e_home),
      e_target(e_home),
      a_grip(options.grip_use * circuit.mu_road * params.g),
      a_brake(options.brake_use * params.F_brake_max / params.m),
      limited(std::isfinite(perception.road_range)) {
    // the profile must hold on whichever lane the car is in
    const std::vector<double> offsets = {e_home, e_pass};
    if (limited) {
        CorneringLimit limit = cornering_limit(c.track, offsets, a_grip, c.v_max);
        k_line = std::move(limit.k_line);
        v_limit = std::move(limit.v);
    } else {
        std::tie(s_grid, v_grid) =
            grip_speed_profile(c.track, offsets, c.mu_road, c.v_max, opt.grip_use, opt.a_accel, a_brake, 0.5, p.g);
    }
}

std::pair<double, double> TrackPurePursuit::speed_reference(double s) const {
    const double lap = c.track.length;
    if (!limited) {
        const double v = interp_periodic(s, s_grid, v_grid, lap);
        return {v, interp_periodic(s + 1.0, s_grid, v_grid, lap) - v};
    }
    const int n = static_cast<int>(v_limit.size());
    const double step = lap / n;
    const int first = static_cast<int>(s / step), count = static_cast<int>(perception.road_range / step) + 1;
    std::vector<double> v(count), k(count);
    for (int i = 0; i < count; ++i) {
        const int j = ((first + i) % n + n) % n;
        v[i] = v_limit[j];
        k[i] = k_line[j];
    }
    const double v_beyond = *std::min_element(v_limit.begin(), v_limit.end());
    const auto [v0, v1] = speed_within_sight(std::move(v), k, step, a_grip, a_brake, v_beyond);
    return {v0, (v1 - v0) / step};
}

bool TrackPurePursuit::traffic_in_the_way(double t, double s, double speed) {
    const double lap = c.track.length;
    for (const Detection& car : perception.observe(t, s)) {
        if (std::fabs(car.e_y - e_home) > 0.5 * c.lane_width) continue;  // not in the home lane
        const double ahead = lap_difference(car.s, s, lap);
        const double reach = c.car_length + opt.pass_time * std::max(speed - car.speed, 0.0);
        if (-opt.window_behind <= ahead && ahead <= std::max(opt.window_ahead, reach)) return true;
    }
    return false;
}

Input TrackPurePursuit::operator()(double t, const State& x_true) {
    const Track& track = c.track;
    const State x = perception.ego(x_true);
    const double s = track.project(x[IX], x[IY]).first;

    // passing rule: slide the target line toward the lane it should be in
    const double goal = traffic_in_the_way(t, s, x[IUX]) ? e_pass : e_home;
    const double dt = t_prev_ ? t - *t_prev_ : 0.0;
    t_prev_ = t;
    e_target += std::clamp(goal - e_target, -opt.lateral_rate * dt, opt.lateral_rate * dt);

    const double psi = x[IPSI];
    const double rear_x = x[IX] - p.b * std::cos(psi), rear_y = x[IY] - p.b * std::sin(psi);
    const double L_d = std::min(std::max(opt.k_lookahead * x[IUX], opt.lookahead_min), perception.road_range);
    const Pose target = track.pose(track.project(rear_x, rear_y).first + L_d, e_target);
    const double alpha = std::atan2(target.y - rear_y, target.x - rear_x) - psi;
    const double chord = std::hypot(target.x - rear_x, target.y - rear_y);
    const double delta_cmd =
        std::clamp(std::atan(2.0 * p.L() * std::sin(alpha) / chord), -p.delta_max, p.delta_max);
    const double rate = (delta_cmd - x[IDELTA]) / opt.tau_steer;

    const auto [v_ref, dv_ds] = speed_reference(s);
    double accel = v_ref * dv_ds + opt.k_speed * (v_ref - x[IUX]);
    if (limited) accel = std::min(accel, opt.a_accel);  // the lap profile has the drive's limit built in; this one has not
    const double drag = 0.5 * p.rho * p.CdA * x[IUX] * x[IUX] + p.Crr * p.m * p.g;
    return {rate, p.m * accel + drag};
}

}  // namespace ov
