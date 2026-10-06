#include "sim/scenarios/overtake/run.hpp"

#include <cmath>
#include <optional>

namespace ov {

std::pair<OvertakeLog, OvertakeResult> run_overtake(const OvertakeScenario& scenario, const ControlLaw& controller,
                                                    PlantModel& plant, double dt_ctrl, double dt_log) {
    const int substeps = static_cast<int>(std::lround(dt_ctrl / dt_log));
    plant.reset(scenario.initial_state());
    double t = 0.0;
    State x = plant.observe();
    OvertakeLog log;
    std::optional<double> returned_since;
    bool done = false;
    while (!done && t < scenario.t_max + scenario.settle_time) {
        const Input u = plant.saturate(controller(t, x));
        for (int i = 0; i < substeps; ++i) {
            log.t.push_back(t);
            log.x.push_back(x);
            log.u.push_back(u);
            if (scenario.clearance(t, x) <= 0.0) {
                done = true;
                break;
            }
            if (!scenario.returned(t, x)) {
                returned_since.reset();
            } else if (!returned_since) {
                returned_since = t;
            } else if (t - *returned_since >= scenario.settle_time - 1e-9) {
                done = true;
                break;
            }
            x = plant.step(u, dt_log);
            t += dt_log;
        }
    }
    return {log, scenario.evaluate(log.t, log.x)};
}

}  // namespace ov
