// What a simulation needs from a controller.
#pragma once

#include "parameters.hpp"

namespace ov {

// Maps the time and the car's state [X, Y, psi, Ux, Uy, r, delta] to the input
// [steering rate, Fx] (parameters.json). It is called once per control period
// and may keep state between calls.
class Controller {
public:
    virtual ~Controller() = default;
    virtual Input operator()(double t, const State& x) = 0;  // input to hold from time t, given the state x
};

}  // namespace ov
