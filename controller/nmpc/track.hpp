// NMPC for a lap of the circuit, on the plant's dynamic bicycle model in track coordinates.
//
// The dynamics of [Ux, Uy, r, delta] and the inputs [delta_rate, Fx] are the
// plant's (`dynamics`). The pose is carried relative to the track instead of in
// the world frame,
//
//     state  [s, e_y, e_psi, Ux, Uy, r, delta]
//     s'     = (Ux cos e_psi - Uy sin e_psi) / (1 - kappa e_y)
//     e_y'   = Ux sin e_psi + Uy cos e_psi
//     e_psi' = r - kappa s'
//
// so corners enter through the curvature kappa and the track edges are bounds
// on e_y. The curvature of each step of the horizon is a parameter, read from
// the track at the positions of the previous plan.
//
// The perception may know the road only a limited distance ahead. Beyond that
// the plan continues with the last known curvature, and its speed is limited to
// `unseen_speed`, so the car always keeps to a speed from which it can brake
// for whatever comes into view.
//
// Speed is not prescribed per corner: the cost asks for the speed limit, and
// the tire model with its friction limit, together with the track-edge and
// sideslip constraints, makes the plan brake for a corner it could not
// otherwise hold. Traffic cars are kept out by the same soft ellipse as on the
// straight road, for the `n_cars` nearest ones the perception reports, each
// predicted to hold its reported speed and lateral offset.
#pragma once

#include <optional>
#include <string>
#include <vector>

#include "controller/interface.hpp"
#include "controller/nmpc/common.hpp"
#include "sim/perception.hpp"

namespace ov {

// `lane` is the lane the car keeps when nothing is in the way. The ellipse
// keeps the bodies `margin_lat` apart side by side and `margin_long` apart nose
// to tail; `edge_margin` is kept between the body and the asphalt edge.
// `perception` supplies the traffic, the ego state and how far ahead the road
// is known; the default reports everything exactly. `unseen_speed` [m/s] is the
// speed limit on the part of the plan beyond the known road; the default is
// what the circuit's tightest corner allows on its centreline at the road's
// friction limit, so that radius is taken as known in advance.
class TrackNMPC : public Controller {
public:
    static constexpr int NS = 4;  // slacks per step: traffic ellipses, track edges, sideslip, speed limit

    struct Options {
        double horizon = 4.0;
        double h = 0.1;
        std::string lane = "right";
        int n_cars = 2;
        double margin_lat = 0.6;
        double margin_long = 1.0;
        double edge_margin = 0.2;
        double sideslip_max = 7.0 * M_PI / 180.0;
        double w_speed = 1.0;
        double w_lane = 0.2;
        double w_lat = 1.0;
        double w_accel = 1.0;
        double w_rate = 100.0;
        double w_slack = 1e3;
        int max_iter = 200;
        std::optional<double> unseen_speed;
    };

    TrackNMPC(const Circuit& circuit, const VehicleParams& params, const Options& options,
              std::optional<Perception> perception = std::nullopt);
    TrackNMPC(const Circuit& circuit, const VehicleParams& params, std::optional<Perception> perception = std::nullopt)
        : TrackNMPC(circuit, params, Options(), std::move(perception)) {}

    Input operator()(double t, const State& x) override;
    // World state as the model's [s, e_y, e_psi, Ux, Uy, r, delta].
    State model_state(const State& x) const;

    Circuit c;
    VehicleParams p;
    Perception perception;
    double h;
    int N, n_cars;
    double unseen_speed;
    double ell_a, ell_b;
    std::vector<double> solve_times;
    int failures = 0;  // solves that did not converge; their result is still applied
    Eigen::MatrixXd plan;  // last predicted states in track coordinates, (N + 1) x NX; empty before the first solve

private:
    // Mean known curvature over each step of the predicted positions `s_plan` (N + 1).
    std::vector<double> step_curvature(const std::vector<double>& s_plan) const;
    // Predicted s (n_cars x N, row per car) and lateral offset of the nearest detected cars, unwrapped around s0.
    std::pair<std::vector<std::vector<double>>, std::vector<double>> traffic(double t, double s0);

    casadi::Function solver_;
    std::vector<double> lbw_, ubw_, lbg_, ubg_;
    nmpc::WarmStart warm_;
};

}  // namespace ov
