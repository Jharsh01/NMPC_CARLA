// Scenario 1. Overtaking benchmark: road layout, lead car, pass criteria and metrics.
//
// Road frame: X along the road, Y to the left. The ego lane is centred on Y = 0
// and the passing lane on Y = lane_width; both lanes run in the same direction.
// The lead car holds the ego-lane centre at constant speed. The ego starts at
// X = 0 with its front bumper `d_trig` behind the lead car's rear bumper, and
// has to pass and return to the ego lane ahead of the lead car.
//
// Both cars are rectangles of the same size. The ego state [X, Y, psi, ...] is
// its centre of gravity, which sits `cg_to_center` ahead of the body centre.
#pragma once

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "parameters.hpp"
#include "sim/geometry.hpp"

namespace ov {

struct OvertakeResult {
    bool passed;
    std::vector<std::string> failures;  // names of the criteria that failed
    double min_clearance;  // smallest distance between the two car bodies [m]
    double road_margin;  // smallest distance from the ego body to a road edge [m], negative if off the road
    double peak_sideslip;  // largest |atan(Uy / Ux)| [rad]
    std::optional<double> t_complete;  // time at which the ego is back in lane and stays there [s]
    std::optional<double> distance;  // distance travelled by the ego up to t_complete [m]
};

struct OvertakeScenario {
    double v0 = 90.0 / 3.6;  // ego entry speed [m/s]
    double v_lead = 50.0 / 3.6;  // lead car speed [m/s]
    double d_trig = 30.0;  // bumper-to-bumper gap at which the overtake starts [m]
    double mu = 0.9;  // road friction coefficient [-]
    double lane_width = 3.5;  // [m]
    double car_length = 4.79;  // CARLA Model 3 bounding box [m]
    double car_width = 2.16;  // CARLA Model 3 bounding box [m]
    double cg_to_center = 0.45;  // centre of gravity ahead of the body centre [m]
    double t_max = 30.0;  // time allowed for the overtake [s]

    // pass criteria
    double clearance_min = 0.3;  // between the car bodies [m]
    double sideslip_max = 10.0 * M_PI / 180.0;  // [rad]
    double return_gap = 10.0;  // ego rear bumper ahead of lead front bumper [m]
    double y_tol = 0.3;  // offset from the ego-lane centre [m]
    double psi_tol = 2.0 * M_PI / 180.0;  // heading [rad]
    double settle_time = 1.0;  // how long the return conditions must hold [s]

    std::pair<double, double> road_edges() const { return {-0.5 * lane_width, 1.5 * lane_width}; }  // right, left
    State initial_state() const;
    double lead_x(double t) const;  // X of the lead car's body centre [m]
    Corners ego_corners(const State& x) const;
    Corners lead_corners(double t) const;
    double clearance(double t, const State& x) const;  // distance between the two bodies [m], 0 on overlap
    double road_margin(const State& x) const;  // ego body to the nearer road edge [m], negative off the road
    double gap_ahead(double t, const State& x) const;  // ego rear bumper minus lead front bumper [m]
    bool returned(double t, const State& x) const;  // ahead of the lead car and aligned with the ego lane
    OvertakeResult evaluate(const std::vector<double>& t, const std::vector<State>& x) const;  // score a logged run
};

// Named cases for the controller comparison, in the order they are run.
//
// relaxed: both should pass; 7.2 s to contact, so a comfortable 2 m/s^2 lane
// change fits with room to spare.
// late_wet: NMPC should pass, pure pursuit should not; 1.2 s to contact on a wet
// road. A smooth quintic lane change that clears the lead car in time needs
// 5.25 m/s^2 > mu*g = 4.9; steering out hard first and finishing the lane
// change later needs about 1.0 s.
// slow_closing: pure pursuit should do as well at a fraction of the compute;
// 18 s alongside the lead car, far longer than a 3-4 s NMPC horizon.
const std::vector<std::pair<std::string, OvertakeScenario>>& overtake_cases();
const OvertakeScenario& overtake_case(const std::string& name);

}  // namespace ov
