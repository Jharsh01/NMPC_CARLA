// What a simulation needs from a plant.
#pragma once

#include "parameters.hpp"

namespace ov {

// A simulated car, driven with the input [steering rate, Fx] and observed as
// the state [X, Y, psi, Ux, Uy, r, delta] (parameters.json).
class PlantModel {
public:
    virtual ~PlantModel() = default;
    virtual void reset(const State& x0) = 0;  // put the car in state x0
    virtual State observe() const = 0;  // the current state
    virtual Input saturate(const Input& u) const = 0;  // u after the car's actuator and force limits
    virtual State step(const Input& u, double duration) = 0;  // hold u for duration seconds, return the new state
    virtual void set_friction(double mu) = 0;  // tire-road friction of the surface under the car
};

}  // namespace ov
