// Stand-in for the Chrono plant when the project is built without Project Chrono.
#include <stdexcept>

#include "plant/chrono_plant.hpp"

namespace ov {

struct ChronoPlant::Impl {};

ChronoPlant::ChronoPlant(const VehicleParams&, double, double) {
    throw std::runtime_error("built without Project Chrono; configure with -DChrono_DIR=<install>/lib/cmake/Chrono");
}
ChronoPlant::~ChronoPlant() = default;
void ChronoPlant::reset(const State&) {}
State ChronoPlant::observe() const { return State::Zero(); }
Input ChronoPlant::saturate(const Input& u) const { return u; }
State ChronoPlant::step(const Input&, double) { return State::Zero(); }
void ChronoPlant::set_friction(double) {}
ChronoPlant::Facts ChronoPlant::facts() const { return {}; }

}  // namespace ov
