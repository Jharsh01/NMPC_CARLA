// NMPC for the overtake on the same dynamic bicycle model the plant integrates.
//
// State and input are the plant's own,
//
//     state  [X, Y, psi, Ux, Uy, r, delta]
//     input  [delta_rate, Fx]
//
// and the prediction model is `dynamics` (Fiala tires with friction-circle
// derating, load transfer), built on CasADi symbols and discretised with one
// RK4 step per interval. The plant adds steering lag and tire relaxation on top
// of this model; the controller does not model those.
//
// The lead car is kept out by an ellipse, the cost holds the entry speed and
// the ego-lane centre and penalises lateral velocity, yaw rate and the inputs.
// The ellipse, the road-edge limits and a sideslip limit are soft, so the
// problem stays feasible when the car is pushed past a limit it planned to ride
// along. It is solved with IPOPT in a receding horizon, warm started from the
// previous solution.
#pragma once

#include <optional>
#include <vector>

#include "controller/interface.hpp"
#include "controller/nmpc/common.hpp"
#include "sim/scenarios/overtake/scenario.hpp"

namespace ov {

// Plans `horizon` seconds ahead in steps of `h`. The ellipse is centred on the
// lead car and is the smallest one of lateral semi-axis `ellipse_width` that
// keeps the two bodies `margin_lat` apart side by side and `margin_long` apart
// nose to tail. `edge_margin` is the distance kept between the body and the
// road edges, `sideslip_use` the share of the scenario's sideslip limit the
// plan may use. The weights apply to speed error [m/s], lane offset [m],
// lateral velocity [m/s] and yaw rate [rad/s], longitudinal acceleration Fx/m
// [m/s^2], steering rate [rad/s] and constraint violation.
class NMPC : public Controller {
public:
    static constexpr int NS = 3;  // slacks per step: lead-car ellipse, road edges, sideslip

    struct Options {
        double horizon = 5.0;
        double h = 0.1;
        double margin_lat = 0.6;
        double margin_long = 1.0;
        std::optional<double> ellipse_width;  // default: lane width - 0.4
        double edge_margin = 0.3;
        double sideslip_use = 0.7;
        double w_speed = 1.0;
        double w_lane = 0.2;
        double w_lat = 1.0;
        double w_accel = 1.0;
        double w_rate = 100.0;
        double w_slack = 1e3;
        int max_iter = 200;
    };

    NMPC(const OvertakeScenario& scenario, const VehicleParams& params, const Options& options);
    NMPC(const OvertakeScenario& scenario, const VehicleParams& params) : NMPC(scenario, params, Options()) {}

    Input operator()(double t, const State& x) override;
    // Constant speed, riding along the top of the ellipse past the lead car.
    std::vector<double> initial_guess(double t, const State& x0) const;

    OvertakeScenario sc;
    VehicleParams p;
    double h;
    int N;
    double ell_a, ell_b, ell_x;  // ellipse semi-axes and its offset from the ego centre of gravity
    std::optional<double> v_des;
    std::vector<double> solve_times;
    int failures = 0;  // solves that did not converge; their result is still applied
    Eigen::MatrixXd plan;  // last predicted states, (N + 1) x NX; empty before the first solve

private:
    casadi::Function solver_;
    std::vector<double> lbw_, ubw_, lbg_, ubg_;
    nmpc::WarmStart warm_;
};

}  // namespace ov
