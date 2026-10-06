// Scenario 2. One-lap circuit: two lanes of asphalt, a grass strip on each side, slower traffic.
//
// The centreline is the lane divider. The circuit is driven clockwise, so the
// right lane (e_y < 0) is on the inside of most corners. Traffic cars have no
// dynamics: each holds a lane centre at constant speed along that lane.
#pragma once

#include <array>
#include <string>
#include <vector>

#include "parameters.hpp"
#include "sim/geometry.hpp"
#include "sim/scenarios/circuit/track.hpp"

namespace ov {

// Centreline corners [m], clockwise from the top-left one, and the radius at each.
// The outline follows the sketch: 300 m top straight; on the right side 30 m
// down, 60 m out to the right, 100 m down, 60 m back to the left and 200 m
// down; 300 m along the bottom and 330 m up the left side.
extern const std::vector<std::array<double, 2>> CORNERS;
extern const std::vector<double> RADII;  // the 30 m leg allows at most 15 m at its two ends
extern const std::array<double, 2> START;  // 100 m after the top-left corner

// Constant-speed car on a lane centre. `at` is a world point near its position at t = 0.
struct TrafficCar {
    std::array<double, 2> at;
    std::string lane = "right";
    double speed = 50.0 / 3.6;  // [m/s]
};

struct TrackState {
    double s, e_y, e_psi;
};

struct TrafficPose {
    double x, y, psi, s;
};

struct Circuit {
    Circuit();

    Track track;
    double lane_width = 3.5;  // [m]
    double grass_width = 1.75;  // half a lane on each side [m]
    double mu_road = 0.9;  // asphalt
    double mu_grass = 0.3;
    double v_max = 80.0 / 3.6;  // ego speed limit [m/s]
    double car_length = 4.79;  // CARLA Model 3 bounding box [m]
    double car_width = 2.16;
    double cg_to_center = 0.45;  // ego centre of gravity ahead of its body centre [m]
    std::vector<TrafficCar> traffic;

    double road_half_width() const { return lane_width; }
    double half_width() const { return lane_width + grass_width; }  // centreline to the outer edge of the grass
    double lane_offset(const std::string& lane) const;  // e_y of a lane centre [m]
    double friction(double e_y) const;  // friction coefficient at lateral offset e_y; nan beyond the grass

    double traffic_s(const TrafficCar& car, double t) const;  // arc length of a traffic car at time t, 0..lap
    TrafficPose traffic_pose(const TrafficCar& car, double t) const;  // body centre and heading at time t, and s

    State initial_state(double speed = 50.0 / 3.6, const std::string& lane = "right") const;
    TrackState track_state(const State& x) const;  // arc length, lateral offset, heading relative to the track
    Corners ego_corners(const State& x) const;
    Corners body_corners(double x, double y, double psi) const;  // a car body centred on (x, y)
    double clearance(double t, const State& x) const;  // ego body to the nearest traffic body [m], 0 on overlap
    double road_margin(const State& x) const;  // ego body to the nearer asphalt edge [m], negative on the grass
};

}  // namespace ov
