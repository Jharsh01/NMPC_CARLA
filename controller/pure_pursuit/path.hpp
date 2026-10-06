// Reference path for the overtake: lane change out, passing lane, lane change back.
//
// The path is a lateral offset Y_ref(X) in the road frame, planned once at the
// trigger from the lead car's position and speed, assuming both cars keep their
// speed. Each lane change is a quintic, Y = w (10 s^3 - 15 s^4 + 6 s^5) with
// s = (X - X_start) / length, which starts and ends with zero heading and
// curvature. Its peak lateral acceleration at speed v is QUINTIC_PEAK * w v^2 / length^2.
#pragma once

#include "sim/scenarios/overtake/scenario.hpp"

namespace ov {

extern const double QUINTIC_PEAK;  // max of the quintic's second derivative, 5.77

double quintic(double s);
double quintic_inverse(double y);  // s in [0, 1] at which quintic(s) = y, by bisection

struct OvertakePath {
    double width;  // lateral offset of the passing lane [m]
    double x_out;  // X where the lane change out starts [m]
    double length_out;  // [m]
    double x_back;  // X where the lane change back starts [m]
    double length_back;  // [m]
    double a_lat_out;  // peak lateral acceleration of the lane change out at the planning speed [m/s^2]
    bool feasible;  // a_lat_out within the road's grip

    double y(double X) const;  // lateral reference Y_ref [m] at road position X
};

// Plan the overtake from ego state `x` at the scenario's start (t = 0).
//
// `a_lat` [m/s^2] is the lateral-acceleration budget for each lane change. The
// lane change out is shortened if that is needed to clear the lead car by
// `scenario.clearance_min` before reaching it, even if the result needs more
// grip than the road has; `feasible` records whether it does. The lane change
// back starts once the ego's rear is `gap_back` ahead of the lead's front.
OvertakePath plan_overtake(const OvertakeScenario& scenario, const State& x, double a_lat = 2.0,
                           double gap_back = 3.0);

}  // namespace ov
