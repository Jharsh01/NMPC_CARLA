// Closed-loop run of a controller on the overtaking scenario.
#pragma once

#include <vector>

#include "plant/interface.hpp"
#include "sim/scenarios/circuit/run.hpp"  // ControlLaw
#include "sim/scenarios/overtake/scenario.hpp"

namespace ov {

// One row per logged sample; u is the saturated input applied from that sample.
struct OvertakeLog {
    std::vector<double> t;
    std::vector<State> x;
    std::vector<Input> u;
};

// Drive `plant` through `scenario` with `controller(t, x) -> u`.
//
// The controller is called every `dt_ctrl` seconds and its input held in
// between; the state is logged every `dt_log` seconds. The run ends at a
// collision, once the ego has returned to its lane for the settle time, or at
// `scenario.t_max`. `plant` should be built with the scenario's friction.
std::pair<OvertakeLog, OvertakeResult> run_overtake(const OvertakeScenario& scenario, const ControlLaw& controller,
                                                    PlantModel& plant, double dt_ctrl = 0.05, double dt_log = 0.01);

}  // namespace ov
