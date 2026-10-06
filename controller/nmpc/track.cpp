#include "controller/nmpc/track.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "plant/integrate.hpp"

namespace ov {

using casadi::Slice;
using casadi::SX;
static constexpr double INF = std::numeric_limits<double>::infinity();
static constexpr double FAR = 1e4;  // s offset that puts an unused traffic slot out of reach [m]

static double tightest_corner_speed(const Circuit& c, const VehicleParams& p) {
    double kappa_max = 0.0;
    for (double kappa : c.track.seg_kappa) kappa_max = std::max(kappa_max, std::fabs(kappa));
    return std::sqrt(c.mu_road * p.g / kappa_max);
}

TrackNMPC::TrackNMPC(const Circuit& circuit, const VehicleParams& params, const Options& o,
                     std::optional<Perception> perception_)
    : c(circuit),
      p(params),
      perception(perception_ ? std::move(*perception_) : Perception(circuit)),
      h(o.h),
      N(static_cast<int>(std::lround(o.horizon / o.h))),
      n_cars(o.n_cars),
      unseen_speed(std::min(o.unseen_speed.value_or(tightest_corner_speed(circuit, params)), circuit.v_max)) {
    // ellipse around a traffic car, in ego centre-of-gravity coordinates
    const double ly = c.car_width + o.margin_lat;
    const double lx = c.car_length + o.margin_long;
    const double B = ell_b = c.lane_width - 0.4;
    const double A = ell_a = lx / std::sqrt(1.0 - (ly / B) * (ly / B));
    const double e_max = c.road_half_width() - 0.5 * c.car_width - o.edge_margin;
    const double to_front = 0.5 * c.car_length - c.cg_to_center;
    const double to_rear = 0.5 * c.car_length + c.cg_to_center;
    const double slip = std::tan(o.sideslip_max);
    const double e_ref = c.lane_offset(o.lane);

    // parameters: measured state, curvature and speed limit of each step,
    // traffic positions at each predicted state
    const SX x_init = SX::sym("x_init", NX);
    const SX kappa = SX::sym("kappa", N);
    const SX v_lim = SX::sym("v_lim", N);
    const SX car_s = SX::sym("car_s", n_cars, N);
    const SX car_e = SX::sym("car_e", n_cars);
    const SX par = SX::vertcat({x_init, kappa, v_lim, vec(car_s), car_e});

    const SX x = SX::sym("x", NX, N + 1);
    const SX u = SX::sym("u", NU, N);  // the longitudinal input is the acceleration Fx / m, for scaling
    const SX slack = SX::sym("s", NS, N);

    const VehicleParams& pp = p;
    SX J = 0.0;
    std::vector<SX> g_eq = {x(Slice(), 0) - x_init};
    std::vector<SX> g_in;  // each entry <= 0
    for (int k = 0; k < N; ++k) {
        const SX xk = x(Slice(), k), uk = u(Slice(), k), nxt = x(Slice(), k + 1), sk = slack(Slice(), k);
        const SX kap = kappa(k);
        auto f = [&pp, &kap](const SX& z, const SX& v) {
            // with psi = e_psi
            const StateOf<SX> d = dynamics<SX>(nmpc::state_of(z), {v(IRATE), pp.m * v(IFX)}, pp);
            const SX s_dot = d[IX] / (1.0 - kap * z(IY));
            return SX::vertcat({s_dot, d[IY], d[IPSI] - kap * s_dot, d[IUX], d[IUY], d[IR], d[IDELTA]});
        };
        g_eq.push_back(nxt - rk4_step(f, xk, uk, h));

        const SX v_lat = nxt(IUX) * sin(nxt(IPSI)) + nxt(IUY) * cos(nxt(IPSI));
        J += o.w_speed * sq(nxt(IUX) - c.v_max) + o.w_lane * sq(nxt(IY) - e_ref) +
             o.w_lat * (sq(v_lat) + sq(nxt(IR) - kap * nxt(IUX))) + o.w_accel * sq(uk(IFX)) +
             o.w_rate * sq(uk(IRATE)) + o.w_slack * (sum1(sk) + sumsqr(sk));

        for (int j = 0; j < n_cars; ++j) {
            const SX ds = nxt(IX) - c.cg_to_center - car_s(j, k);
            g_in.push_back(1.0 - sq(ds / A) - sq((nxt(IY) - car_e(j)) / B) - sk(0));
        }
        for (double arm : {to_front, -to_rear}) {  // asphalt edges, at the front and rear ends of the body
            const SX e_end = nxt(IY) + arm * sin(nxt(IPSI));
            g_in.push_back(e_end - e_max - sk(1));
            g_in.push_back(-e_max - e_end - sk(1));
        }
        g_in.push_back(nxt(IUY) - slip * nxt(IUX) - sk(2));
        g_in.push_back(-nxt(IUY) - slip * nxt(IUX) - sk(2));
        g_in.push_back(nxt(IUX) - v_lim(k) - sk(3));
    }

    const SX eq = SX::vertcat(g_eq), in = SX::vertcat(g_in);
    lbg_.assign(eq.size1(), 0.0);
    lbg_.insert(lbg_.end(), in.size1(), -INF);
    ubg_.assign(eq.size1() + in.size1(), 0.0);

    for (int k = 0; k < N; ++k) {  // force limits are set per solve
        lbw_.insert(lbw_.end(), {-p.delta_rate_max, 0.0});
        ubw_.insert(ubw_.end(), {p.delta_rate_max, 0.0});
    }
    for (int k = 0; k <= N; ++k) {
        for (int i = 0; i < NX; ++i) {
            const bool bounded = k > 0;
            // the tire model is not meant for speeds below 3 m/s
            lbw_.push_back(bounded && i == IDELTA ? -p.delta_max : bounded && i == IUX ? 3.0 : -INF);
            ubw_.push_back(bounded && i == IDELTA ? p.delta_max : INF);
        }
    }
    lbw_.insert(lbw_.end(), NS * N, 0.0);
    ubw_.insert(ubw_.end(), NS * N, INF);

    const casadi::SXDict nlp = {
        {"x", SX::vertcat({vec(u), vec(x), vec(slack)})}, {"p", par}, {"f", J}, {"g", SX::vertcat({eq, in})}};
    solver_ = nmpc::make_solver("nmpc_track", nlp, o.max_iter);
}

State TrackNMPC::model_state(const State& x) const {
    State z = x;
    const TrackState ts = c.track_state(x);
    z[IX] = ts.s;
    z[IY] = ts.e_y;
    z[IPSI] = ts.e_psi;
    return z;
}

std::vector<double> TrackNMPC::step_curvature(const std::vector<double>& s_plan) const {
    std::vector<double> out(N);
    for (int k = 0; k < N; ++k) {
        double sum = 0.0;
        for (int i = 0; i < 5; ++i) {
            const double w = i / 4.0;
            sum += perception.curvature(s_plan[0], s_plan[k] * (1.0 - w) + s_plan[k + 1] * w);
        }
        out[k] = sum / 5.0;
    }
    return out;
}

std::pair<std::vector<std::vector<double>>, std::vector<double>> TrackNMPC::traffic(double t, double s0) {
    const Track& track = c.track;
    const double lap = track.length;
    struct Slot {
        double distance;
        std::vector<double> s;
        double e_y;
    };
    std::vector<Slot> slots;
    for (const Detection& car : perception.observe(t, s0)) {
        // constant speed along the line at the car's lateral offset
        const double start = track.offset_length(car.s, car.e_y);
        std::vector<double> s(N);
        for (int k = 0; k < N; ++k) s[k] = track.offset_to_s(start + car.speed * h * (k + 1), car.e_y);
        const double ahead = lap_difference(s[0], s0, lap);  // signed distance to the car, along the track
        // unwrap the lap wrap out of s - s[0], then place it around s0
        std::vector<double> unwrapped(N);
        double previous = 0.0, offset = 0.0;
        for (int k = 0; k < N; ++k) {
            const double value = s[k] - s[0];
            const double jump = value - previous;
            if (jump > 0.5 * lap) offset -= lap;
            if (jump < -0.5 * lap) offset += lap;
            previous = value;
            unwrapped[k] = s0 + ahead + value + offset;
        }
        slots.push_back({std::fabs(ahead), std::move(unwrapped), car.e_y});
    }
    std::stable_sort(slots.begin(), slots.end(), [](const Slot& a, const Slot& b) { return a.distance < b.distance; });
    slots.resize(std::min<size_t>(slots.size(), n_cars));
    while (static_cast<int>(slots.size()) < n_cars) slots.push_back({0.0, std::vector<double>(N, s0 + FAR), 0.0});

    std::vector<std::vector<double>> car_s;
    std::vector<double> car_e;
    for (Slot& slot : slots) {
        car_s.push_back(std::move(slot.s));
        car_e.push_back(slot.e_y);
    }
    return {car_s, car_e};
}

Input TrackNMPC::operator()(double t, const State& x_true) {
    const State x = perception.ego(x_true);
    const State z0 = model_state(x);

    std::vector<double> s_plan(N + 1);
    for (int k = 0; k <= N; ++k)  // constant speed at first, then the previous plan's progress from where the car is now
        s_plan[k] = z0[IX] + (plan.size() ? plan(k, IX) - plan(0, IX) : z0[IUX] * h * k);
    const auto [car_s, car_e] = traffic(t, z0[IX]);

    std::vector<double> par(z0.data(), z0.data() + NX);
    const std::vector<double> kappa = step_curvature(s_plan);
    par.insert(par.end(), kappa.begin(), kappa.end());
    for (int k = 0; k < N; ++k) par.push_back(perception.road_known(z0[IX], s_plan[k + 1]) ? c.v_max : unseen_speed);
    for (int k = 0; k < N; ++k)
        for (int j = 0; j < n_cars; ++j) par.push_back(car_s[j][k]);
    par.insert(par.end(), car_e.begin(), car_e.end());

    const auto [fx_min, fx_max] = fx_limits(x[IUX], p);
    for (int k = 0; k < N; ++k) {
        lbw_[NU * k + IFX] = fx_min / p.m;
        ubw_[NU * k + IFX] = fx_max / p.m;
    }

    if (!warm_.set) {
        warm_.x0.assign(NU * N, 0.0);
        for (int k = 0; k <= N; ++k) {
            State guess = z0;
            guess[IX] = s_plan[k];
            if (k > 0) guess[IPSI] = guess[IUY] = guess[IR] = guess[IDELTA] = 0.0;
            warm_.x0.insert(warm_.x0.end(), guess.data(), guess.data() + NX);
        }
        warm_.x0.insert(warm_.x0.end(), NS * N, 0.0);
    } else {
        // the warm start's s values belong to the previous lap position; move them to this one
        const double shift = z0[IX] - warm_.x0[NU * N + IX];
        for (int k = 0; k <= N; ++k) warm_.x0[NU * N + NX * k + IX] += shift;
    }

    const std::vector<double> w = nmpc::solve(
        solver_, {{"p", par}, {"lbx", lbw_}, {"ubx", ubw_}, {"lbg", lbg_}, {"ubg", ubg_}}, warm_, solve_times,
        failures);

    plan = Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        w.data() + NU * N, N + 1, NX);
    return {w[IRATE], p.m * w[IFX]};
}

}  // namespace ov
