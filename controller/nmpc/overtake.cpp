#include "controller/nmpc/overtake.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "plant/integrate.hpp"

namespace ov {

using casadi::Slice;
using casadi::SX;
static constexpr double INF = std::numeric_limits<double>::infinity();

NMPC::NMPC(const OvertakeScenario& scenario, const VehicleParams& params, const Options& o)
    : sc(scenario), p(params), h(o.h), N(static_cast<int>(std::lround(o.horizon / o.h))) {
    // ellipse around the lead car, in ego centre-of-gravity coordinates
    const double ly = sc.car_width + o.margin_lat;
    const double lx = sc.car_length + o.margin_long;
    const double B = ell_b = o.ellipse_width.value_or(sc.lane_width - 0.4);
    if (B <= ly) throw std::invalid_argument("ellipse_width must exceed car_width + margin_lat");
    const double A = ell_a = lx / std::sqrt(1.0 - (ly / B) * (ly / B));
    ell_x = sc.cg_to_center;  // the ego body centre is this far behind its centre of gravity

    const auto [right, left] = sc.road_edges();
    const double y_min = right + 0.5 * sc.car_width + o.edge_margin;
    const double y_max = left - 0.5 * sc.car_width - o.edge_margin;
    const double to_front = 0.5 * sc.car_length - sc.cg_to_center;
    const double to_rear = 0.5 * sc.car_length + sc.cg_to_center;
    const double slip = std::tan(o.sideslip_use * sc.sideslip_max);

    // parameters: measured state, lead car position now, desired speed
    const SX x_init = SX::sym("x_init", NX);
    const SX lead_now = SX::sym("lead_now");
    const SX v_des_ = SX::sym("v_des");
    const SX par = SX::vertcat({x_init, lead_now, v_des_});

    // decision variables; the longitudinal input is the acceleration Fx / m, for scaling
    const SX x = SX::sym("x", NX, N + 1);
    const SX u = SX::sym("u", NU, N);
    const SX slack = SX::sym("s", NS, N);

    const VehicleParams& pp = p;
    auto f = [&pp](const SX& xk, const SX& uk) {
        return nmpc::column(dynamics<SX>(nmpc::state_of(xk), {uk(IRATE), pp.m * uk(IFX)}, pp));
    };

    SX J = 0.0;
    std::vector<SX> g_eq = {x(Slice(), 0) - x_init};
    std::vector<SX> g_in;  // each entry <= 0
    for (int k = 0; k < N; ++k) {
        const SX xk = x(Slice(), k), uk = u(Slice(), k), nxt = x(Slice(), k + 1), sk = slack(Slice(), k);
        g_eq.push_back(nxt - rk4_step(f, xk, uk, h));

        const SX v_lat = nxt(IUX) * sin(nxt(IPSI)) + nxt(IUY) * cos(nxt(IPSI));
        J += o.w_speed * sq(nxt(IUX) - v_des_) + o.w_lane * sq(nxt(IY)) + o.w_lat * (sq(v_lat) + sq(nxt(IR))) +
             o.w_accel * sq(uk(IFX)) + o.w_rate * sq(uk(IRATE)) + o.w_slack * (sum1(sk) + sumsqr(sk));

        // the lead car, at the predicted state
        const SX dx = nxt(IX) - (lead_now + sc.v_lead * h * (k + 1)) - ell_x;
        g_in.push_back(1.0 - sq(dx / A) - sq(nxt(IY) / B) - sk(0));
        // road edges, at the front and rear ends of the body
        for (double arm : {to_front, -to_rear}) {
            const SX y_end = nxt(IY) + arm * sin(nxt(IPSI));
            g_in.push_back(y_end - y_max - sk(1));
            g_in.push_back(y_min - y_end - sk(1));
        }
        // sideslip
        g_in.push_back(nxt(IUY) - slip * nxt(IUX) - sk(2));
        g_in.push_back(-nxt(IUY) - slip * nxt(IUX) - sk(2));
    }

    const SX eq = SX::vertcat(g_eq), in = SX::vertcat(g_in);
    lbg_.assign(eq.size1(), 0.0);
    lbg_.insert(lbg_.end(), in.size1(), -INF);
    ubg_.assign(eq.size1() + in.size1(), 0.0);

    // simple bounds: steering rate (force limits are set per solve), steering angle, slacks
    for (int k = 0; k < N; ++k) {
        lbw_.insert(lbw_.end(), {-p.delta_rate_max, 0.0});
        ubw_.insert(ubw_.end(), {p.delta_rate_max, 0.0});
    }
    for (int k = 0; k <= N; ++k) {
        for (int i = 0; i < NX; ++i) {
            const bool bounded = k > 0;
            lbw_.push_back(bounded && i == IDELTA ? -p.delta_max : bounded && i == IUX ? p.Ux_min : -INF);
            ubw_.push_back(bounded && i == IDELTA ? p.delta_max : INF);
        }
    }
    lbw_.insert(lbw_.end(), NS * N, 0.0);
    ubw_.insert(ubw_.end(), NS * N, INF);

    const casadi::SXDict nlp = {
        {"x", SX::vertcat({vec(u), vec(x), vec(slack)})}, {"p", par}, {"f", J}, {"g", SX::vertcat({eq, in})}};
    solver_ = nmpc::make_solver("nmpc", nlp, o.max_iter);
}

std::vector<double> NMPC::initial_guess(double t, const State& x0) const {
    std::vector<double> X(N + 1), y(N + 1);
    for (int k = 0; k <= N; ++k) {
        X[k] = x0[IX] + x0[IUX] * h * k;
        const double dx = X[k] - sc.lead_x(t + h * k) - ell_x;
        y[k] = std::max(x0[IY], 1.05 * ell_b * std::sqrt(std::max(1.0 - (dx / ell_a) * (dx / ell_a), 0.0)));
    }
    y[0] = x0[IY];

    std::vector<double> guess(NU * N, 0.0);
    for (int k = 0; k <= N; ++k) {
        State xk = x0;
        xk[IX] = X[k];
        xk[IY] = y[k];
        if (k > 0) {
            // slope of y, central differences inside and one-sided at the end (as numpy.gradient)
            const double dy = k < N ? (y[k + 1] - y[k - 1]) / (2.0 * h) : (y[k] - y[k - 1]) / h;
            xk[IPSI] = std::atan2(dy, x0[IUX]);
            xk[IUY] = xk[IR] = xk[IDELTA] = 0.0;
        }
        guess.insert(guess.end(), xk.data(), xk.data() + NX);
    }
    guess.insert(guess.end(), NS * N, 0.0);
    return guess;
}

Input NMPC::operator()(double t, const State& x) {
    if (!v_des) v_des = x[IUX];  // hold the entry speed, like the baseline
    std::vector<double> par(x.data(), x.data() + NX);
    par.push_back(sc.lead_x(t));
    par.push_back(*v_des);

    // force limits at the current speed, as accelerations
    const auto [fx_min, fx_max] = fx_limits(x[IUX], p);
    for (int k = 0; k < N; ++k) {
        lbw_[NU * k + IFX] = fx_min / p.m;
        ubw_[NU * k + IFX] = fx_max / p.m;
    }

    if (!warm_.set) warm_.x0 = initial_guess(t, x);
    const std::vector<double> w = nmpc::solve(
        solver_, {{"p", par}, {"lbx", lbw_}, {"ubx", ubw_}, {"lbg", lbg_}, {"ubg", ubg_}}, warm_, solve_times,
        failures);

    plan = Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        w.data() + NU * N, N + 1, NX);
    return {w[IRATE], p.m * w[IFX]};
}

}  // namespace ov
