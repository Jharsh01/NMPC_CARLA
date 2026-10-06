// Pure-pursuit baseline for the circuit: follow a lane, a speed profile and a passing rule.
//
// Steering is the pure-pursuit law of the overtake baseline on a lane line of
// the track. Speed follows a grip-limited profile (speed_profile.hpp): computed
// once for the lap if the whole road is known, otherwise at every call over the
// road the perception knows. Pure pursuit has no notion of other cars, so
// passing is a rule: move to the other lane while a traffic car in the home
// lane is within a window around the ego, and come back afterwards.
#pragma once

#include <optional>
#include <string>
#include <vector>

#include "controller/interface.hpp"
#include "sim/perception.hpp"

namespace ov {

// The lookahead point is on the current target line, L_d = max(k_lookahead *
// U_x, lookahead_min) further along the track than the rear axle. The target
// line moves between lane centres at `lateral_rate` [m/s]. A traffic car
// triggers the move when it is between `window_behind` behind and
// `window_ahead` ahead of the ego (centre to centre, along the track). The
// window ahead grows with the closing speed, so that `pass_time` seconds are
// left to change lane before reaching the car. `perception` supplies the
// traffic, the ego state and how far ahead the road is known; the default
// reports everything exactly. `grip_use` is the share of mu*g the speed profile
// may use and `brake_use` the share of the brakes' peak force it plans with.
//
// With a limited road range the car keeps to a speed from which it can brake,
// within the known road, to the speed of the circuit's tightest corner: the
// tightest radius is taken as known in advance (a property of the road).
class TrackPurePursuit : public Controller {
public:
    struct Options {
        std::string lane = "right";
        double grip_use = 0.4;
        double a_accel = 3.0;
        double brake_use = 0.85;
        double k_lookahead = 0.6;
        double lookahead_min = 5.0;
        double tau_steer = 0.05;
        double k_speed = 3.0;
        double window_ahead = 30.0;
        double window_behind = 15.0;
        double pass_time = 3.5;
        double lateral_rate = 1.2;
    };

    TrackPurePursuit(const Circuit& circuit, const VehicleParams& params, const Options& options,
                     std::optional<Perception> perception = std::nullopt);
    TrackPurePursuit(const Circuit& circuit, const VehicleParams& params,
                     std::optional<Perception> perception = std::nullopt)
        : TrackPurePursuit(circuit, params, Options(), std::move(perception)) {}

    Input operator()(double t, const State& x) override;
    // Profile speed [m/s] and its slope dv/ds [1/s] at arc length `s`.
    std::pair<double, double> speed_reference(double s) const;

    Circuit c;
    VehicleParams p;
    Options opt;
    Perception perception;
    double e_home, e_pass, e_target;
    double a_grip, a_brake;
    bool limited;  // the road is known only a limited distance ahead
    std::vector<double> k_line, v_limit;  // cornering limit round the lap (limited road range)
    std::vector<double> s_grid, v_grid;  // speed profile of the lap (whole road known)

private:
    bool traffic_in_the_way(double t, double s, double speed);
    std::optional<double> t_prev_;
};

}  // namespace ov
