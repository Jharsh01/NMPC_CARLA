// Vehicle parameters and the state and input definitions, read from parameters.json.
#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>

namespace ov {

constexpr int NX = 7;  // state [X, Y, psi, Ux, Uy, r, delta]
constexpr int NU = 2;  // input [delta_rate, Fx]
enum StateIndex { IX, IY, IPSI, IUX, IUY, IR, IDELTA };
enum InputIndex { IRATE, IFX };

using State = Eigen::Matrix<double, NX, 1>;
using Input = Eigen::Matrix<double, NU, 1>;

// Single-track model parameters. The fields, their defaults, units and where
// each value comes from are in parameters.json. Cornering stiffnesses are per
// axle (both tires).
struct VehicleParams {
    double m, Izz, a, b, h, C_alpha_f, C_alpha_r, mu, drive_front, brake_front, F_drive_max, F_brake_max, P_max,
        CdA, Crr, rho, delta_max, delta_rate_max, Ux_min, g;

    VehicleParams();  // the values in parameters.json
    static VehicleParams from_file(const std::string& path);

    double L() const { return a + b; }  // wheelbase [m]
    double understeer_gradient() const;  // linear range [rad per m/s^2]

    struct Field {
        const char* name;
        double VehicleParams::* member;
    };
    static const std::vector<Field>& fields();

private:
    struct Unset {};
    explicit VehicleParams(Unset) {}
};

// Path of the parameters.json the defaults are read from.
std::string parameters_path();
// Path of the parameters identified for the Chrono sedan (written by identify_chrono).
std::string chrono_parameters_path();

}  // namespace ov
