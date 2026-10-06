// Pieces shared by the two NMPC formulations.
#pragma once

#include <casadi/casadi.hpp>
#include <chrono>
#include <vector>

#include "plant/bicycle.hpp"

namespace ov::nmpc {

inline StateOf<casadi::SX> state_of(const casadi::SX& x) {
    return {x(IX), x(IY), x(IPSI), x(IUX), x(IUY), x(IR), x(IDELTA)};
}

inline casadi::SX column(const StateOf<casadi::SX>& x) {
    return casadi::SX::vertcat(std::vector<casadi::SX>(x.begin(), x.end()));
}

// IPOPT through CasADi, quiet.
inline casadi::Function make_solver(const std::string& name, const casadi::SXDict& nlp, int max_iter) {
    const casadi::Dict options = {
        {"print_time", false},
        {"ipopt.print_level", 0},
        {"ipopt.sb", "yes"},
        {"ipopt.max_iter", max_iter},
        {"ipopt.tol", 1e-6},
    };
    return casadi::nlpsol(name, "ipopt", nlp, options);
}

// Previous primal and dual solution, to start the next solve from.
struct WarmStart {
    std::vector<double> x0;
    casadi::DM lam_x0, lam_g0;
    bool set = false;
};

// Solve, record the time taken and whether IPOPT converged, keep the solution as the next warm start.
inline std::vector<double> solve(casadi::Function& solver, casadi::DMDict args, WarmStart& warm,
                                 std::vector<double>& solve_times, int& failures) {
    args["x0"] = warm.x0;
    if (warm.set) {
        args["lam_x0"] = warm.lam_x0;
        args["lam_g0"] = warm.lam_g0;
    }
    const auto start = std::chrono::steady_clock::now();
    const casadi::DMDict sol = solver(args);
    solve_times.push_back(std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count());
    if (!solver.stats().at("success").as_bool()) ++failures;
    warm.x0 = sol.at("x").get_elements();
    warm.lam_x0 = sol.at("lam_x");
    warm.lam_g0 = sol.at("lam_g");
    warm.set = true;
    return warm.x0;
}

}  // namespace ov::nmpc
