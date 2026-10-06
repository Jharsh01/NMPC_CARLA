#include "sim/scenarios/overtake/scenario.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ov {

State OvertakeScenario::initial_state() const {
    State x = State::Zero();
    x[IUX] = v0;
    return x;
}

double OvertakeScenario::lead_x(double t) const {
    const double ego_front = 0.5 * car_length - cg_to_center;
    return ego_front + d_trig + 0.5 * car_length + v_lead * t;
}

Corners OvertakeScenario::ego_corners(const State& x) const {
    const double c = std::cos(x[IPSI]), s = std::sin(x[IPSI]);
    return body_corners(x[IX] - cg_to_center * c, x[IY] - cg_to_center * s, x[IPSI], car_length, car_width);
}

Corners OvertakeScenario::lead_corners(double t) const {
    return body_corners(lead_x(t), 0.0, 0.0, car_length, car_width);
}

double OvertakeScenario::clearance(double t, const State& x) const {
    return polygon_distance(ego_corners(x), lead_corners(t));
}

double OvertakeScenario::road_margin(const State& x) const {
    const Corners corners = ego_corners(x);
    const auto y = corners.col(1);
    const auto [right, left] = road_edges();
    return std::min(y.minCoeff() - right, left - y.maxCoeff());
}

double OvertakeScenario::gap_ahead(double t, const State& x) const {
    return ego_corners(x).col(0).minCoeff() - (lead_x(t) + 0.5 * car_length);
}

bool OvertakeScenario::returned(double t, const State& x) const {
    return gap_ahead(t, x) >= return_gap && std::fabs(x[IY]) <= y_tol && std::fabs(x[IPSI]) <= psi_tol;
}

OvertakeResult OvertakeScenario::evaluate(const std::vector<double>& t, const std::vector<State>& x) const {
    OvertakeResult r{};
    r.min_clearance = INFINITY;
    r.road_margin = INFINITY;
    r.peak_sideslip = 0.0;
    for (size_t k = 0; k < t.size(); ++k) {
        r.min_clearance = std::min(r.min_clearance, clearance(t[k], x[k]));
        r.road_margin = std::min(r.road_margin, road_margin(x[k]));
        r.peak_sideslip = std::max(r.peak_sideslip, std::fabs(std::atan2(x[k][IUY], x[k][IUX])));
    }

    std::optional<size_t> start;  // first sample of the current run of `returned`
    for (size_t k = 0; k < t.size(); ++k) {
        if (!returned(t[k], x[k])) {
            start.reset();
            continue;
        }
        if (!start) start = k;
        if (t[k] - t[*start] >= settle_time - 1e-9) {
            r.t_complete = t[*start];
            r.distance = x[*start][IX] - x[0][IX];
            break;
        }
    }

    if (!(r.min_clearance >= clearance_min)) r.failures.push_back("clearance");
    if (!(r.road_margin >= 0.0)) r.failures.push_back("road");
    if (!(r.peak_sideslip <= sideslip_max)) r.failures.push_back("sideslip");
    if (!(r.t_complete && *r.t_complete <= t_max)) r.failures.push_back("completed");
    r.passed = r.failures.empty();
    return r;
}

const std::vector<std::pair<std::string, OvertakeScenario>>& overtake_cases() {
    static const std::vector<std::pair<std::string, OvertakeScenario>> cases = [] {
        OvertakeScenario relaxed, late_wet, slow_closing;
        relaxed.v0 = 80.0 / 3.6;
        relaxed.d_trig = 60.0;
        late_wet.v0 = 110.0 / 3.6;
        late_wet.d_trig = 20.0;
        late_wet.mu = 0.5;
        slow_closing.v0 = 60.0 / 3.6;
        slow_closing.d_trig = 30.0;
        return std::vector<std::pair<std::string, OvertakeScenario>>{
            {"nominal", OvertakeScenario()},
            {"relaxed", relaxed},
            {"late_wet", late_wet},
            {"slow_closing", slow_closing},
        };
    }();
    return cases;
}

const OvertakeScenario& overtake_case(const std::string& name) {
    for (const auto& [key, scenario] : overtake_cases())
        if (key == name) return scenario;
    throw std::invalid_argument("unknown overtake case: " + name);
}

}  // namespace ov
