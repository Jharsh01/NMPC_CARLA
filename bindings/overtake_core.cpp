// Python module `overtake_core`: the C++ model, plants, controllers and scenarios,
// for the tests and the animations.
#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <casadi/casadi.hpp>

#include "controller/nmpc/overtake.hpp"
#include "controller/nmpc/track.hpp"
#include "controller/pure_pursuit/overtake.hpp"
#include "controller/pure_pursuit/speed_profile.hpp"
#include "controller/pure_pursuit/track.hpp"
#include "plant/bicycle.hpp"
#include "plant/bicycle_plant.hpp"
#include "plant/chrono_plant.hpp"
#include "sim/perception.hpp"
#include "sim/scenarios/circuit/run.hpp"
#include "sim/scenarios/overtake/run.hpp"

namespace py = pybind11;
using namespace ov;
using namespace pybind11::literals;

namespace {

constexpr double INF = std::numeric_limits<double>::infinity();

// Apply f: double -> K values over a float or an array; returns K floats or K arrays of the same shape.
template <size_t K, class F>
py::tuple over(const py::object& arg, F f) {
    const auto in = py::array_t<double, py::array::c_style | py::array::forcecast>::ensure(arg);
    if (!in) throw py::type_error("expected a number or an array of numbers");
    py::tuple out(K);
    if (in.ndim() == 0) {
        const std::array<double, K> value = f(*in.data());
        for (size_t j = 0; j < K; ++j) out[j] = value[j];
        return out;
    }
    std::vector<py::array_t<double>> arrays;
    for (size_t j = 0; j < K; ++j)
        arrays.emplace_back(std::vector<py::ssize_t>(in.shape(), in.shape() + in.ndim()));
    for (py::ssize_t i = 0; i < in.size(); ++i) {
        const std::array<double, K> value = f(in.data()[i]);
        for (size_t j = 0; j < K; ++j) arrays[j].mutable_data()[i] = value[j];
    }
    for (size_t j = 0; j < K; ++j) out[j] = arrays[j];
    return out;
}

Eigen::MatrixXd rows(const std::vector<State>& x) {
    Eigen::MatrixXd out(x.size(), NX);
    for (size_t k = 0; k < x.size(); ++k) out.row(k) = x[k];
    return out;
}

Eigen::MatrixXd rows(const std::vector<Input>& u) {
    Eigen::MatrixXd out(u.size(), NU);
    for (size_t k = 0; k < u.size(); ++k) out.row(k) = u[k];
    return out;
}

Eigen::VectorXd column(const std::vector<double>& v) { return Eigen::Map<const Eigen::VectorXd>(v.data(), v.size()); }

template <class T>
py::object or_none(const std::optional<T>& value) {
    return value ? py::cast(*value) : py::object(py::none());
}

// A copy of `object` with the given attributes changed (what dataclasses.replace does for a dataclass).
py::object replace(const py::object& object, const py::kwargs& changes) {
    py::object copy = py::module_::import("copy").attr("copy")(object);
    for (const auto& item : changes) py::setattr(copy, item.first, item.second);
    return copy;
}

// The model's derivative evaluated through its CasADi symbols, to check them against the numbers.
State dynamics_casadi(const State& x, const Input& u, const VehicleParams& p) {
    using casadi::SX;
    const SX xs = SX::sym("x", NX), us = SX::sym("u", NU);
    const SX f = nmpc::column(dynamics<SX>(nmpc::state_of(xs), {us(IRATE), us(IFX)}, p));
    const casadi::Function function("f", {xs, us}, {f});
    const std::vector<casadi::DM> out = function(std::vector<casadi::DM>{
        std::vector<double>(x.data(), x.data() + NX), std::vector<double>(u.data(), u.data() + NU)});
    return State(out[0].get_elements().data());
}

ControlLaw law(Controller& controller) { return std::ref(controller); }

}  // namespace

PYBIND11_MODULE(overtake_core, m) {
    m.doc() = "C++ vehicle model, plants, controllers and scenarios of the overtaking project";

    // ---- parameters and state definition
    m.attr("NX") = NX;
    m.attr("NU") = NU;
    m.attr("IX") = int(IX);
    m.attr("IY") = int(IY);
    m.attr("IPSI") = int(IPSI);
    m.attr("IUX") = int(IUX);
    m.attr("IUY") = int(IUY);
    m.attr("IR") = int(IR);
    m.attr("IDELTA") = int(IDELTA);
    m.attr("IRATE") = int(IRATE);
    m.attr("IFX") = int(IFX);
    m.attr("PARAMETERS_PATH") = parameters_path();
    m.attr("CHRONO_PARAMETERS_PATH") = chrono_parameters_path();
    m.def("replace", &replace);

    py::class_<VehicleParams> params(m, "VehicleParams");
    params.def(py::init<>())
        .def_static("from_file", &VehicleParams::from_file)
        .def_property_readonly("L", &VehicleParams::L)
        .def_property_readonly("understeer_gradient", &VehicleParams::understeer_gradient)
        .def("__copy__", [](const VehicleParams& self) { return VehicleParams(self); });
    for (const auto& field : VehicleParams::fields()) params.def_readwrite(field.name, field.member);

    // ---- plant model
    m.attr("F_MIN") = F_MIN;
    m.def("fiala_lateral", py::vectorize([](double alpha, double Fz, double Fx, double C_alpha, double mu) {
              return fiala_lateral<double>(alpha, Fz, Fx, C_alpha, mu);
          }));
    py::class_<AxleForces<double>>(m, "AxleForces")
        .def_readonly("Fzf", &AxleForces<double>::Fzf)
        .def_readonly("Fzr", &AxleForces<double>::Fzr)
        .def_readonly("Fxf", &AxleForces<double>::Fxf)
        .def_readonly("Fxr", &AxleForces<double>::Fxr)
        .def_readonly("Fyf", &AxleForces<double>::Fyf)
        .def_readonly("Fyr", &AxleForces<double>::Fyr)
        .def_readonly("alpha_f", &AxleForces<double>::alpha_f)
        .def_readonly("alpha_r", &AxleForces<double>::alpha_r);
    m.def("dynamics", [](const State& x, const Input& u, const VehicleParams& p) { return dynamics(x, u, p); });
    m.def("dynamics_casadi", &dynamics_casadi);
    m.def("axle_forces", [](const State& x, const Input& u, const VehicleParams& p) { return axle_forces(x, u, p); });
    m.def("fx_limits", &fx_limits);

    py::class_<PlantModel>(m, "PlantModel")
        .def("reset", &PlantModel::reset)
        .def("observe", &PlantModel::observe)
        .def("saturate", &PlantModel::saturate)
        .def("step", &PlantModel::step)
        .def("set_friction", &PlantModel::set_friction);
    py::class_<BicyclePlant, PlantModel>(m, "BicyclePlant")
        .def(py::init<const VehicleParams&, double, double, double>(), "params"_a, "dt"_a = 1e-3,
             "tau_steer"_a = 0.05, "relaxation_length"_a = 0.3)
        .def_readwrite("p", &BicyclePlant::p);
    py::class_<ChronoPlant, PlantModel>(m, "ChronoPlant")
        .def(py::init<const VehicleParams&, double, double>(), "params"_a, "dt"_a = 1e-3, "tau_steer"_a = 0.05)
        .def("facts", [](const ChronoPlant& plant) {
            const ChronoPlant::Facts f = plant.facts();
            return py::dict("mass"_a = f.mass, "Izz"_a = f.Izz, "a"_a = f.a, "b"_a = f.b, "h"_a = f.h,
                            "tire_radius"_a = f.tire_radius, "steer_gain"_a = f.steer_gain,
                            "brake_force_max"_a = f.brake_force_max);
        });

    // ---- scenario 1: overtake
    py::class_<OvertakeResult>(m, "OvertakeResult")
        .def_readonly("passed", &OvertakeResult::passed)
        .def_property_readonly("failures", [](const OvertakeResult& r) { return py::tuple(py::cast(r.failures)); })
        .def_readonly("min_clearance", &OvertakeResult::min_clearance)
        .def_readonly("road_margin", &OvertakeResult::road_margin)
        .def_readonly("peak_sideslip", &OvertakeResult::peak_sideslip)
        .def_property_readonly("t_complete", [](const OvertakeResult& r) { return or_none(r.t_complete); })
        .def_property_readonly("distance", [](const OvertakeResult& r) { return or_none(r.distance); })
        .def("__repr__", [](const OvertakeResult& r) {
            std::string text = r.passed ? "OvertakeResult(passed" : "OvertakeResult(failed:";
            for (const std::string& name : r.failures) text += " " + name;
            return text + ")";
        });

    using Sc = OvertakeScenario;
    py::class_<Sc>(m, "OvertakeScenario")
        .def(py::init<>())
        .def("__copy__", [](const Sc& self) { return Sc(self); })
        .def_readwrite("v0", &Sc::v0)
        .def_readwrite("v_lead", &Sc::v_lead)
        .def_readwrite("d_trig", &Sc::d_trig)
        .def_readwrite("mu", &Sc::mu)
        .def_readwrite("lane_width", &Sc::lane_width)
        .def_readwrite("car_length", &Sc::car_length)
        .def_readwrite("car_width", &Sc::car_width)
        .def_readwrite("cg_to_center", &Sc::cg_to_center)
        .def_readwrite("t_max", &Sc::t_max)
        .def_readwrite("clearance_min", &Sc::clearance_min)
        .def_readwrite("sideslip_max", &Sc::sideslip_max)
        .def_readwrite("return_gap", &Sc::return_gap)
        .def_readwrite("y_tol", &Sc::y_tol)
        .def_readwrite("psi_tol", &Sc::psi_tol)
        .def_readwrite("settle_time", &Sc::settle_time)
        .def_property_readonly("road_edges", &Sc::road_edges)
        .def("initial_state", &Sc::initial_state)
        .def("lead_x", py::vectorize([](const Sc* self, double t) { return self->lead_x(t); }))
        .def("ego_corners", &Sc::ego_corners)
        .def("lead_corners", &Sc::lead_corners)
        .def("clearance", &Sc::clearance)
        .def("road_margin", &Sc::road_margin)
        .def("gap_ahead", &Sc::gap_ahead)
        .def("returned", &Sc::returned)
        .def("evaluate", [](const Sc& self, const std::vector<double>& t, const Eigen::Ref<const Eigen::MatrixXd>& x) {
            std::vector<State> states;
            for (Eigen::Index k = 0; k < x.rows(); ++k) states.emplace_back(x.row(k).transpose());
            return self.evaluate(t, states);
        });
    py::dict cases;
    for (const auto& [name, scenario] : overtake_cases()) cases[py::str(name)] = scenario;
    m.attr("SCENARIOS") = cases;

    py::class_<OvertakeLog>(m, "OvertakeLog")
        .def_property_readonly("t", [](const OvertakeLog& log) { return column(log.t); })
        .def_property_readonly("x", [](const OvertakeLog& log) { return rows(log.x); })
        .def_property_readonly("u", [](const OvertakeLog& log) { return rows(log.u); });
    m.def("run_overtake",
          [](const Sc& sc, Controller& controller, PlantModel& plant, double dt_ctrl, double dt_log) {
              return run_overtake(sc, law(controller), plant, dt_ctrl, dt_log);
          },
          "scenario"_a, "controller"_a, "plant"_a, "dt_ctrl"_a = 0.05, "dt_log"_a = 0.01);
    m.def("run_overtake",
          [](const Sc& sc, const ControlLaw& controller, PlantModel& plant, double dt_ctrl, double dt_log) {
              return run_overtake(sc, controller, plant, dt_ctrl, dt_log);
          },
          "scenario"_a, "controller"_a, "plant"_a, "dt_ctrl"_a = 0.05, "dt_log"_a = 0.01);

    // ---- scenario 2: circuit
    m.attr("CORNERS") = CORNERS;
    m.attr("RADII") = RADII;
    m.attr("START") = START;

    py::class_<Track>(m, "Track")
        .def(py::init<const std::vector<std::array<double, 2>>&, const std::vector<double>&,
                      const std::array<double, 2>&>())
        .def_readonly("length", &Track::length)
        .def_property_readonly("seg_length", [](const Track& t) { return column(t.seg_length); })
        .def_property_readonly("seg_kappa", [](const Track& t) { return column(t.seg_kappa); })
        .def_property_readonly("seg_s", [](const Track& t) { return column(t.seg_s); })
        .def("curvature", py::vectorize([](const Track* self, double s) { return self->curvature(s); }))
        .def("pose",
             [](const Track& self, const py::object& s, double e_y) {
                 return over<3>(s, [&](double value) {
                     const Pose p = self.pose(value, e_y);
                     return std::array<double, 3>{p.x, p.y, p.psi};
                 });
             },
             "s"_a, "e_y"_a = 0.0)
        .def("project", &Track::project)
        .def("offset_length", py::vectorize([](const Track* self, double s, double e_y) {
                 return self->offset_length(s, e_y);
             }))
        .def("offset_to_s", py::vectorize([](const Track* self, double distance, double e_y) {
                 return self->offset_to_s(distance, e_y);
             }));

    py::class_<TrafficCar>(m, "TrafficCar")
        .def(py::init([](std::array<double, 2> at, std::string lane, double speed) {
                 return TrafficCar{at, std::move(lane), speed};
             }),
             "at"_a, "lane"_a = "right", "speed"_a = 50.0 / 3.6)
        .def_readwrite("at", &TrafficCar::at)
        .def_readwrite("lane", &TrafficCar::lane)
        .def_readwrite("speed", &TrafficCar::speed);

    py::class_<Circuit>(m, "Circuit")
        .def(py::init<>())
        .def("__copy__", [](const Circuit& self) { return Circuit(self); })
        .def_readonly("track", &Circuit::track)
        .def_readwrite("lane_width", &Circuit::lane_width)
        .def_readwrite("grass_width", &Circuit::grass_width)
        .def_readwrite("mu_road", &Circuit::mu_road)
        .def_readwrite("mu_grass", &Circuit::mu_grass)
        .def_readwrite("v_max", &Circuit::v_max)
        .def_readwrite("car_length", &Circuit::car_length)
        .def_readwrite("car_width", &Circuit::car_width)
        .def_readwrite("cg_to_center", &Circuit::cg_to_center)
        .def_readwrite("traffic", &Circuit::traffic)
        .def_property_readonly("road_half_width", &Circuit::road_half_width)
        .def_property_readonly("half_width", &Circuit::half_width)
        .def("lane_offset", &Circuit::lane_offset)
        .def("friction", py::vectorize([](const Circuit* self, double e_y) { return self->friction(e_y); }))
        .def("traffic_s", py::vectorize([](const Circuit* self, const TrafficCar* car, double t) {
                 return self->traffic_s(*car, t);
             }))
        .def("traffic_pose",
             [](const Circuit& self, const TrafficCar& car, double t) {
                 const TrafficPose p = self.traffic_pose(car, t);
                 return std::make_tuple(p.x, p.y, p.psi, p.s);
             })
        .def("initial_state", &Circuit::initial_state, "speed"_a = 50.0 / 3.6, "lane"_a = "right")
        .def("track_state",
             [](const Circuit& self, const State& x) {
                 const TrackState ts = self.track_state(x);
                 return std::make_tuple(ts.s, ts.e_y, ts.e_psi);
             })
        .def("ego_corners", &Circuit::ego_corners)
        .def("body_corners", &Circuit::body_corners)
        .def("clearance", &Circuit::clearance)
        .def("road_margin", &Circuit::road_margin);

    py::class_<LapResult>(m, "LapResult")
        .def_readonly("completed", &LapResult::completed)
        .def_property_readonly("lap_time", [](const LapResult& r) { return or_none(r.lap_time); })
        .def_readonly("collided", &LapResult::collided)
        .def_readonly("left_track", &LapResult::left_track)
        .def_readonly("min_clearance", &LapResult::min_clearance)
        .def_readonly("road_margin", &LapResult::road_margin)
        .def_readonly("time_on_grass", &LapResult::time_on_grass)
        .def_readonly("peak_sideslip", &LapResult::peak_sideslip)
        .def_readonly("min_speed", &LapResult::min_speed)
        .def_readonly("max_speed", &LapResult::max_speed);
    py::class_<LapLog>(m, "LapLog")
        .def_property_readonly("t", [](const LapLog& log) { return column(log.t); })
        .def_property_readonly("x", [](const LapLog& log) { return rows(log.x); })
        .def_property_readonly("u", [](const LapLog& log) { return rows(log.u); })
        .def_property_readonly("s", [](const LapLog& log) { return column(log.s); })
        .def_property_readonly("e_y", [](const LapLog& log) { return column(log.e_y); })
        .def_property_readonly("mu", [](const LapLog& log) { return column(log.mu); });
    auto lap = [](const Circuit& c, const ControlLaw& controller, PlantModel& plant, const std::optional<State>& x0,
                  double t_max, double dt_ctrl, double dt_log) {
        return run_lap(c, controller, plant, x0 ? &*x0 : nullptr, t_max, dt_ctrl, dt_log);
    };
    m.def("run_lap",
          [lap](const Circuit& c, Controller& controller, PlantModel& plant, const std::optional<State>& x0,
                double t_max, double dt_ctrl, double dt_log) {
              return lap(c, law(controller), plant, x0, t_max, dt_ctrl, dt_log);
          },
          "circuit"_a, "controller"_a, "plant"_a, "x0"_a = py::none(), "t_max"_a = 180.0, "dt_ctrl"_a = 0.05,
          "dt_log"_a = 0.01);
    m.def("run_lap", lap, "circuit"_a, "controller"_a, "plant"_a, "x0"_a = py::none(), "t_max"_a = 180.0,
          "dt_ctrl"_a = 0.05, "dt_log"_a = 0.01);

    // ---- perception
    m.attr("EGO_SIGMA") = EGO_SIGMA;
    py::class_<Detection>(m, "Detection")
        .def_readonly("s", &Detection::s)
        .def_readonly("e_y", &Detection::e_y)
        .def_readonly("speed", &Detection::speed);
    py::class_<Perception>(m, "Perception")
        .def(py::init<const Circuit&, double, double, double, double, double, unsigned, std::optional<State>,
                      double>(),
             "circuit"_a, "range_ahead"_a = INF, "range_behind"_a = INF, "sigma_s"_a = 0.0, "sigma_e"_a = 0.0,
             "sigma_v"_a = 0.0, "seed"_a = 0, "sigma_ego"_a = py::none(), "road_range"_a = INF)
        .def_static("limited", &Perception::limited, "circuit"_a, "seed"_a = 0)
        .def("observe", &Perception::observe)
        .def("ego", &Perception::ego)
        .def("road_known", py::vectorize([](const Perception* self, double s_ego, double s) {
                 return self->road_known(s_ego, s);
             }))
        .def("curvature", py::vectorize([](const Perception* self, double s_ego, double s) {
                 return self->curvature(s_ego, s);
             }))
        .def_readonly("road_range", &Perception::road_range);

    // ---- controllers
    py::class_<Controller>(m, "Controller")
        .def("__call__", [](Controller& self, double t, const State& x) { return self(t, x); });

    m.attr("QUINTIC_PEAK") = QUINTIC_PEAK;
    m.def("quintic", py::vectorize(&quintic));
    m.def("quintic_inverse", &quintic_inverse);
    py::class_<OvertakePath>(m, "OvertakePath")
        .def(py::init([](double width, double x_out, double length_out, double x_back, double length_back,
                         double a_lat_out, bool feasible) {
                 return OvertakePath{width, x_out, length_out, x_back, length_back, a_lat_out, feasible};
             }),
             "width"_a, "x_out"_a, "length_out"_a, "x_back"_a, "length_back"_a, "a_lat_out"_a, "feasible"_a)
        .def_readonly("width", &OvertakePath::width)
        .def_readonly("x_out", &OvertakePath::x_out)
        .def_readonly("length_out", &OvertakePath::length_out)
        .def_readonly("x_back", &OvertakePath::x_back)
        .def_readonly("length_back", &OvertakePath::length_back)
        .def_readonly("a_lat_out", &OvertakePath::a_lat_out)
        .def_readonly("feasible", &OvertakePath::feasible)
        .def("y", py::vectorize([](const OvertakePath* self, double X) { return self->y(X); }));
    m.def("plan_overtake", &plan_overtake, "scenario"_a, "x"_a, "a_lat"_a = 2.0, "gap_back"_a = 3.0);

    py::class_<PurePursuit, Controller>(m, "PurePursuit")
        .def(py::init<const Sc&, const VehicleParams&, double, double, double, double, double>(), "scenario"_a,
             "params"_a, "k_lookahead"_a = 1.4, "lookahead_min"_a = 5.0, "tau_steer"_a = 0.05, "k_speed"_a = 1.0,
             "a_lat"_a = 2.0)
        .def_property_readonly("path", [](const PurePursuit& c) { return or_none(c.path); });

    py::class_<NMPC, Controller>(m, "NMPC")
        .def(py::init([](const Sc& sc, const VehicleParams& p, double horizon, double h, double margin_lat,
                         double margin_long, std::optional<double> ellipse_width, double edge_margin,
                         double sideslip_use, double w_speed, double w_lane, double w_lat, double w_accel,
                         double w_rate, double w_slack, int max_iter) {
                 NMPC::Options o;
                 o.horizon = horizon, o.h = h, o.margin_lat = margin_lat, o.margin_long = margin_long;
                 o.ellipse_width = ellipse_width, o.edge_margin = edge_margin, o.sideslip_use = sideslip_use;
                 o.w_speed = w_speed, o.w_lane = w_lane, o.w_lat = w_lat, o.w_accel = w_accel, o.w_rate = w_rate;
                 o.w_slack = w_slack, o.max_iter = max_iter;
                 return std::make_unique<NMPC>(sc, p, o);
             }),
             "scenario"_a, "params"_a, "horizon"_a = 5.0, "h"_a = 0.1, "margin_lat"_a = 0.6, "margin_long"_a = 1.0,
             "ellipse_width"_a = py::none(), "edge_margin"_a = 0.3, "sideslip_use"_a = 0.7, "w_speed"_a = 1.0,
             "w_lane"_a = 0.2, "w_lat"_a = 1.0, "w_accel"_a = 1.0, "w_rate"_a = 100.0, "w_slack"_a = 1e3,
             "max_iter"_a = 200)
        .def_readonly("N", &NMPC::N)
        .def_readonly("h", &NMPC::h)
        .def_readonly("ell_a", &NMPC::ell_a)
        .def_readonly("ell_b", &NMPC::ell_b)
        .def_readonly("ell_x", &NMPC::ell_x)
        .def_readonly("failures", &NMPC::failures)
        .def_readonly("solve_times", &NMPC::solve_times)
        .def_readonly("plan", &NMPC::plan);

    m.def(
        "grip_speed_profile",
        [](const Track& track, const std::vector<double>& offsets, double mu, double v_max, double grip_use,
           double a_accel, double a_brake, double ds, double g) {
            const auto [s, v] = grip_speed_profile(track, offsets, mu, v_max, grip_use, a_accel, a_brake, ds, g);
            return std::make_pair(column(s), column(v));
        },
        "track"_a, "offsets"_a, "mu"_a, "v_max"_a, "grip_use"_a = 0.8, "a_accel"_a = 3.0, "a_brake"_a = INF,
        "ds"_a = 0.5, "g"_a = 9.81);

    py::class_<TrackPurePursuit, Controller>(m, "TrackPurePursuit")
        .def(py::init([](const Circuit& c, const VehicleParams& p, std::optional<Perception> perception,
                         std::string lane, double grip_use, double a_accel, double brake_use, double k_lookahead,
                         double lookahead_min, double tau_steer, double k_speed, double window_ahead,
                         double window_behind, double pass_time, double lateral_rate) {
                 TrackPurePursuit::Options o;
                 o.lane = std::move(lane), o.grip_use = grip_use, o.a_accel = a_accel, o.brake_use = brake_use;
                 o.k_lookahead = k_lookahead, o.lookahead_min = lookahead_min, o.tau_steer = tau_steer;
                 o.k_speed = k_speed, o.window_ahead = window_ahead, o.window_behind = window_behind;
                 o.pass_time = pass_time, o.lateral_rate = lateral_rate;
                 return std::make_unique<TrackPurePursuit>(c, p, o, std::move(perception));
             }),
             "circuit"_a, "params"_a, "perception"_a = py::none(), "lane"_a = "right", "grip_use"_a = 0.4,
             "a_accel"_a = 3.0, "brake_use"_a = 0.85, "k_lookahead"_a = 0.6, "lookahead_min"_a = 5.0,
             "tau_steer"_a = 0.05, "k_speed"_a = 3.0, "window_ahead"_a = 30.0, "window_behind"_a = 15.0,
             "pass_time"_a = 3.5, "lateral_rate"_a = 1.2)
        .def("speed_reference",
             [](const TrackPurePursuit& self, const py::object& s) {
                 return over<2>(s, [&](double value) {
                     const auto [v, slope] = self.speed_reference(value);
                     return std::array<double, 2>{v, slope};
                 });
             })
        .def_readonly("a_brake", &TrackPurePursuit::a_brake)
        .def_readonly("a_grip", &TrackPurePursuit::a_grip)
        .def_readonly("e_target", &TrackPurePursuit::e_target)
        .def_property_readonly("v_limit", [](const TrackPurePursuit& c) { return column(c.v_limit); });

    py::class_<TrackNMPC, Controller>(m, "TrackNMPC")
        .def(py::init([](const Circuit& c, const VehicleParams& p, std::optional<Perception> perception,
                         double horizon, double h, std::string lane, int n_cars, double margin_lat,
                         double margin_long, double edge_margin, double sideslip_max, double w_speed, double w_lane,
                         double w_lat, double w_accel, double w_rate, double w_slack, int max_iter,
                         std::optional<double> unseen_speed) {
                 TrackNMPC::Options o;
                 o.horizon = horizon, o.h = h, o.lane = std::move(lane), o.n_cars = n_cars;
                 o.margin_lat = margin_lat, o.margin_long = margin_long, o.edge_margin = edge_margin;
                 o.sideslip_max = sideslip_max, o.w_speed = w_speed, o.w_lane = w_lane, o.w_lat = w_lat;
                 o.w_accel = w_accel, o.w_rate = w_rate, o.w_slack = w_slack, o.max_iter = max_iter;
                 o.unseen_speed = unseen_speed;
                 return std::make_unique<TrackNMPC>(c, p, o, std::move(perception));
             }),
             "circuit"_a, "params"_a, "perception"_a = py::none(), "horizon"_a = 4.0, "h"_a = 0.1,
             "lane"_a = "right", "n_cars"_a = 2, "margin_lat"_a = 0.6, "margin_long"_a = 1.0, "edge_margin"_a = 0.2,
             "sideslip_max"_a = 7.0 * M_PI / 180.0, "w_speed"_a = 1.0, "w_lane"_a = 0.2, "w_lat"_a = 1.0,
             "w_accel"_a = 1.0, "w_rate"_a = 100.0, "w_slack"_a = 1e3, "max_iter"_a = 200,
             "unseen_speed"_a = py::none())
        .def("model_state", &TrackNMPC::model_state)
        .def_readonly("N", &TrackNMPC::N)
        .def_readonly("h", &TrackNMPC::h)
        .def_readonly("unseen_speed", &TrackNMPC::unseen_speed)
        .def_readonly("failures", &TrackNMPC::failures)
        .def_readonly("solve_times", &TrackNMPC::solve_times)
        .def_readonly("plan", &TrackNMPC::plan);
}
