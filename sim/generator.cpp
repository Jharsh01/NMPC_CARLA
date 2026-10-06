// Simulation generator: build a scenario, a plant and the controllers, run them, report and write the logs.
//
//   simulate circuit                        scenario 2: both controllers round the circuit
//   simulate circuit --controller nmpc      or pure_pursuit
//   simulate circuit --perfect              exact traffic and ego state, whole road known
//   simulate circuit --road-range 20 --ego-noise 0
//   simulate overtake nominal               scenario 1, one case (or: all)
//   simulate ... --plant chrono             drive the Project Chrono sedan instead of the bicycle plant; the
//                                           controllers then use parameters_chrono_sedan.json
//   simulate ... --params FILE              vehicle parameters for the controllers and the plant's limits
//   simulate ... --out DIR                  where the logs go (default results/logs)
//
// Each run writes <out>/<scenario>/<controller>.csv (one row per 10 ms) and a
// .json with its result; `python -m sim.view` animates them.
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <vector>

#include "controller/nmpc/overtake.hpp"
#include "controller/nmpc/track.hpp"
#include "controller/pure_pursuit/overtake.hpp"
#include "controller/pure_pursuit/track.hpp"
#include "plant/bicycle_plant.hpp"
#include "plant/chrono_plant.hpp"
#include "sim/perception.hpp"
#include "sim/scenarios/circuit/run.hpp"
#include "sim/scenarios/overtake/run.hpp"

using namespace ov;
using nlohmann::json;
namespace fs = std::filesystem;

namespace {

struct Args {
    std::string scenario;
    std::string overtake_case = "nominal";
    std::string controller = "both";
    std::string plant = "bicycle";
    std::string out = "results/logs";
    std::string params;  // file of vehicle parameters; empty: the one that goes with the plant
    bool perfect = false;
    double range = 60.0, noise = 1.0, ego_noise = 1.0, road_range = 40.0;
    unsigned seed = 0;
};

const char* USAGE =
    "usage: simulate circuit|overtake [case|all] [--controller both|nmpc|pure_pursuit] [--plant bicycle|chrono]\n"
    "                [--params FILE] [--out DIR] [--perfect] [--range M] [--noise K] [--ego-noise K] [--road-range M] [--seed N]\n";

Args parse(int argc, char** argv) {
    Args a;
    std::vector<std::string> words(argv + 1, argv + argc);
    if (words.empty() || words[0] == "-h" || words[0] == "--help") {
        std::cout << USAGE;
        std::exit(words.empty() ? 1 : 0);
    }
    a.scenario = words[0];
    if (a.scenario != "circuit" && a.scenario != "overtake") throw std::invalid_argument("unknown scenario " + a.scenario);
    for (size_t i = 1; i < words.size(); ++i) {
        const std::string& w = words[i];
        auto value = [&]() -> const std::string& {
            if (i + 1 >= words.size()) throw std::invalid_argument(w + " needs a value");
            return words[++i];
        };
        if (w == "--controller") a.controller = value();
        else if (w == "--plant") a.plant = value();
        else if (w == "--out") a.out = value();
        else if (w == "--params") a.params = value();
        else if (w == "--perfect") a.perfect = true;
        else if (w == "--range") a.range = std::stod(value());
        else if (w == "--noise") a.noise = std::stod(value());
        else if (w == "--ego-noise") a.ego_noise = std::stod(value());
        else if (w == "--road-range") a.road_range = std::stod(value());
        else if (w == "--seed") a.seed = static_cast<unsigned>(std::stoul(value()));
        else if (w.rfind("--", 0) != 0 && a.scenario == "overtake") a.overtake_case = w;
        else throw std::invalid_argument("unknown option " + w);
    }
    if (a.controller != "both" && a.controller != "nmpc" && a.controller != "pure_pursuit")
        throw std::invalid_argument("unknown controller " + a.controller);
    return a;
}

// The controllers' model of the car: parameters.json for the bicycle plant, the identified set for Chrono.
VehicleParams vehicle_params(const Args& a) {
    if (!a.params.empty()) return VehicleParams::from_file(a.params);
    if (a.plant == "chrono") return VehicleParams::from_file(chrono_parameters_path());
    return VehicleParams();
}

bool chosen(const Args& a, const std::string& key) { return a.controller == "both" || a.controller == key; }

std::unique_ptr<PlantModel> make_plant(const std::string& name, const VehicleParams& params) {
    if (name == "bicycle") return std::make_unique<BicyclePlant>(params);
    if (name == "chrono") return std::make_unique<ChronoPlant>(params);
    throw std::invalid_argument("unknown plant " + name);
}

std::string solve_times(const std::vector<double>& times, int failures) {
    double sum = 0.0, max = 0.0;
    for (double t : times) {
        sum += t;
        max = std::max(max, t);
    }
    char text[128];
    std::snprintf(text, sizeof text, "solve mean %.0f ms, max %.0f ms, %d not converged", 1e3 * sum / times.size(),
                  1e3 * max, failures);
    return text;
}

json optional(const std::optional<double>& value) { return value ? json(*value) : json(nullptr); }

template <class Log>
void write_log(const fs::path& file, const Log& log, const std::vector<std::string>& extra_names,
               const std::vector<const std::vector<double>*>& extra) {
    fs::create_directories(file.parent_path());
    std::ofstream out(file);
    out << "t,X,Y,psi,Ux,Uy,r,delta,delta_rate,Fx";
    for (const std::string& name : extra_names) out << ',' << name;
    out << '\n' << std::setprecision(12);
    for (size_t k = 0; k < log.t.size(); ++k) {
        out << log.t[k];
        for (int i = 0; i < NX; ++i) out << ',' << log.x[k][i];
        for (int i = 0; i < NU; ++i) out << ',' << log.u[k][i];
        for (const auto* column : extra) out << ',' << (*column)[k];
        out << '\n';
    }
}

void write_result(fs::path file, const json& result) {
    std::ofstream(file) << result.dump(2) << '\n';
    std::cout << "wrote " << file.replace_extension(".csv").string() << std::endl;
}

// ---------------------------------------------------------------- scenario 1

void run_overtake_case(const Args& a, const std::string& name) {
    const OvertakeScenario& sc = overtake_case(name);
    VehicleParams params = vehicle_params(a);
    if (a.plant == "bicycle" && a.params.empty())
        params.mu = sc.mu;
    else  // the scenario's friction relative to the dry road the parameters were identified on
        params.mu *= sc.mu / OvertakeScenario().mu;
    auto plant_on_road = [&] {
        auto plant = make_plant(a.plant, params);
        plant->set_friction(sc.mu);
        return plant;
    };
    const fs::path dir = fs::path(a.out) / name;

    auto report = [&](const std::string& label, const OvertakeResult& r, const std::string& extra) {
        std::string verdict = "PASS";
        if (!r.passed) {
            verdict = "FAIL ";
            for (size_t i = 0; i < r.failures.size(); ++i) verdict += (i ? "," : "") + r.failures[i];
        }
        char completed[32] = "-";
        if (r.t_complete) std::snprintf(completed, sizeof completed, "%.1f s", *r.t_complete);
        std::printf("%-13s %-13s %-34s clearance %5.2f m  road margin %5.2f m  sideslip %4.1f deg  completed %7s  | %s\n",
                    name.c_str(), label.c_str(), verdict.c_str(), r.min_clearance, r.road_margin,
                    r.peak_sideslip * 180.0 / M_PI, completed, extra.c_str());
    };
    auto result_json = [&](const std::string& label, const OvertakeResult& r) {
        return json{{"scenario", "overtake"}, {"case", name}, {"controller", label}, {"plant", a.plant},
                    {"passed", r.passed}, {"failures", r.failures}, {"min_clearance", r.min_clearance},
                    {"road_margin", r.road_margin}, {"peak_sideslip", r.peak_sideslip},
                    {"t_complete", optional(r.t_complete)}, {"distance", optional(r.distance)}};
    };

    if (chosen(a, "pure_pursuit")) {
        PurePursuit controller(sc, params);
        auto plant = plant_on_road();
        const auto [log, r] = run_overtake(sc, std::ref(controller), *plant);
        const OvertakePath& path = *controller.path;
        char extra[128];
        std::snprintf(extra, sizeof extra, "path out %3.0f m, peak %.2f m/s^2 (%s mu*g)", path.length_out,
                      path.a_lat_out, path.feasible ? "within" : "EXCEEDS");
        report("pure pursuit", r, extra);
        json result = result_json("pure pursuit", r);
        result["path"] = {{"width", path.width}, {"x_out", path.x_out}, {"length_out", path.length_out},
                          {"x_back", path.x_back}, {"length_back", path.length_back},
                          {"a_lat_out", path.a_lat_out}, {"feasible", path.feasible}};
        write_log(dir / "pure_pursuit.csv", log, {}, {});
        write_result(dir / "pure_pursuit.json", result);
    }
    if (chosen(a, "nmpc")) {
        NMPC controller(sc, params);
        auto plant = plant_on_road();
        const auto [log, r] = run_overtake(sc, std::ref(controller), *plant);
        report("NMPC", r, solve_times(controller.solve_times, controller.failures));
        write_log(dir / "nmpc.csv", log, {}, {});
        write_result(dir / "nmpc.json", result_json("NMPC", r));
    }
}

// ---------------------------------------------------------------- scenario 2

// Number of traffic cars the ego got ahead of during the run.
int passed(const Circuit& circuit, const LapLog& log) {
    const double lap = circuit.track.length;
    int count = 0;
    for (const TrafficCar& car : circuit.traffic) {
        // every car starts somewhere ahead on the lap
        const double ahead = pymod(circuit.traffic_s(car, log.t.front()) - log.s.front(), lap);
        const double car_travel = car.speed * (log.t.back() - log.t.front());
        count += log.s.back() - log.s.front() > ahead + car_travel;
    }
    return count;
}

void run_circuit(const Args& a) {
    const Circuit circuit;
    const VehicleParams params = vehicle_params(a);
    const fs::path dir = fs::path(a.out) / "circuit";

    auto perception = [&] {
        if (a.perfect) return Perception(circuit);
        return Perception(circuit, a.range, 20.0, 0.5 * a.noise, 0.2 * a.noise, 0.5 * a.noise, a.seed,
                          State(a.ego_noise * EGO_SIGMA), a.road_range);
    };
    auto finish = [&](const std::string& label, const std::string& file, const LapLog& log, const LapResult& r,
                      const std::string& extra) {
        char outcome[32] = "   none";
        if (r.completed) std::snprintf(outcome, sizeof outcome, "%6.1f s", *r.lap_time);
        const int overtaken = passed(circuit, log);
        std::printf("%-13s lap %s   speed %2.0f-%2.0f km/h   passed %d/%zu   clearance %5.2f m   asphalt margin %5.2f m"
                    "   on grass %3.1f s   sideslip %4.1f deg%s",
                    label.c_str(), outcome, 3.6 * r.min_speed, 3.6 * r.max_speed, overtaken, circuit.traffic.size(),
                    r.min_clearance, r.road_margin, r.time_on_grass, r.peak_sideslip * 180.0 / M_PI, extra.c_str());
        if (r.collided || r.left_track)
            std::printf("   %s at %.1f s", r.collided ? "collision" : "left the track", log.t.back());
        std::printf("\n");
        write_log(dir / (file + ".csv"), log, {"s", "e_y", "mu"}, {&log.s, &log.e_y, &log.mu});
        write_result(dir / (file + ".json"),
                     json{{"scenario", "circuit"}, {"controller", label}, {"plant", a.plant},
                          {"completed", r.completed}, {"lap_time", optional(r.lap_time)}, {"collided", r.collided},
                          {"left_track", r.left_track}, {"min_clearance", r.min_clearance},
                          {"road_margin", r.road_margin}, {"time_on_grass", r.time_on_grass},
                          {"peak_sideslip", r.peak_sideslip}, {"min_speed", r.min_speed},
                          {"max_speed", r.max_speed}, {"passed", overtaken}});
    };

    if (chosen(a, "nmpc")) {
        std::printf("driving the lap with NMPC ...\n");
        std::fflush(stdout);
        TrackNMPC controller(circuit, params, perception());
        auto plant = make_plant(a.plant, params);
        const auto [log, r] = run_lap(circuit, std::ref(controller), *plant);
        finish("NMPC", "nmpc", log, r, "   | " + solve_times(controller.solve_times, controller.failures));
    }
    if (chosen(a, "pure_pursuit")) {
        std::printf("driving the lap with pure pursuit ...\n");
        std::fflush(stdout);
        TrackPurePursuit controller(circuit, params, perception());
        auto plant = make_plant(a.plant, params);
        const auto [log, r] = run_lap(circuit, std::ref(controller), *plant);
        finish("pure pursuit", "pure_pursuit", log, r, "");
    }
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const Args a = parse(argc, argv);
        if (a.scenario == "circuit") {
            run_circuit(a);
        } else if (a.overtake_case == "all") {
            for (const auto& [name, scenario] : overtake_cases()) run_overtake_case(a, name);
        } else {
            run_overtake_case(a, a.overtake_case);
        }
    } catch (const std::exception& error) {
        std::cerr << "simulate: " << error.what() << '\n' << USAGE;
        return 1;
    }
    return 0;
}
