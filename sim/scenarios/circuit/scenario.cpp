#include "sim/scenarios/circuit/scenario.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace ov {

const std::vector<std::array<double, 2>> CORNERS = {
    {0.0, 0.0},     {300.0, 0.0},    {300.0, -30.0},  {360.0, -30.0},
    {360.0, -130.0}, {300.0, -130.0}, {300.0, -330.0}, {0.0, -330.0},
};
const std::vector<double> RADII = {40.0, 15.0, 15.0, 25.0, 25.0, 25.0, 30.0, 30.0};
const std::array<double, 2> START = {100.0, 0.0};

Circuit::Circuit()
    : track(CORNERS, RADII, START),
      traffic{
          // slow car 60 m past the start line: every controller catches it on the top straight
          {{160.0, 0.0}, "right", 30.0 / 3.6},
          {{180.0, 0.0}},  // top straight
          {{360.0, -80.0}},  // 100 m straight on the right side
          {{300.0, -220.0}},  // 200 m straight
          {{160.0, -330.0}},  // bottom straight
          {{0.0, -180.0}},  // left side
      } {}

double Circuit::lane_offset(const std::string& lane) const {
    if (lane == "left") return 0.5 * lane_width;
    if (lane == "right") return -0.5 * lane_width;
    throw std::invalid_argument("lane must be 'left' or 'right'");
}

double Circuit::friction(double e_y) const {
    const double e = std::fabs(e_y);
    if (e <= road_half_width()) return mu_road;
    if (e <= half_width()) return mu_grass;
    return std::numeric_limits<double>::quiet_NaN();
}

double Circuit::traffic_s(const TrafficCar& car, double t) const {
    const double e_y = lane_offset(car.lane);
    // distance along its lane from the start line to where the car is at t = 0
    const double start = track.offset_length(track.project(car.at[0], car.at[1]).first, e_y);
    return track.offset_to_s(start + car.speed * t, e_y);
}

TrafficPose Circuit::traffic_pose(const TrafficCar& car, double t) const {
    const double s = traffic_s(car, t);
    const Pose p = track.pose(s, lane_offset(car.lane));
    return {p.x, p.y, p.psi, s};
}

State Circuit::initial_state(double speed, const std::string& lane) const {
    const Pose p = track.pose(0.0, lane_offset(lane));
    State x = State::Zero();
    x[IX] = p.x;
    x[IY] = p.y;
    x[IPSI] = p.psi;
    x[IUX] = speed;
    return x;
}

TrackState Circuit::track_state(const State& x) const {
    const auto [s, e_y] = track.project(x[IX], x[IY]);
    return {s, e_y, wrap_angle(x[IPSI] - track.pose(s).psi)};
}

Corners Circuit::body_corners(double x, double y, double psi) const {
    return ov::body_corners(x, y, psi, car_length, car_width);
}

Corners Circuit::ego_corners(const State& x) const {
    const double c = std::cos(x[IPSI]), s = std::sin(x[IPSI]);
    return body_corners(x[IX] - cg_to_center * c, x[IY] - cg_to_center * s, x[IPSI]);
}

double Circuit::clearance(double t, const State& x) const {
    const Corners ego = ego_corners(x);
    double best = std::numeric_limits<double>::infinity();
    for (const TrafficCar& car : traffic) {
        const TrafficPose p = traffic_pose(car, t);
        const double gap = std::hypot(p.x - x[IX], p.y - x[IY]) - car_length;
        if (gap < best)  // only then can the bodies be closer than the best so far
            best = std::min(best, polygon_distance(ego, body_corners(p.x, p.y, p.psi)));
    }
    return best;
}

double Circuit::road_margin(const State& x) const {
    const Corners corners = ego_corners(x);
    double furthest = 0.0;
    for (int i = 0; i < 4; ++i)
        furthest = std::max(furthest, std::fabs(track.project(corners(i, 0), corners(i, 1)).second));
    return road_half_width() - furthest;
}

}  // namespace ov
