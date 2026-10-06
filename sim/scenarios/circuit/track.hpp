// Closed track built from straights and circular arcs.
//
// The centreline is a polygon whose corners are rounded with a radius each. It
// is parametrised by arc length s [m] from the start line, in the direction of
// travel. Track coordinates are (s, e_y) with e_y the lateral offset from the
// centreline, positive to the left of the direction of travel. Curvature is
// positive in left turns.
#pragma once

#include <array>
#include <utility>
#include <vector>

namespace ov {

struct Pose {
    double x, y, psi;
};

class Track {
public:
    // `corners` are the polygon vertices in driving order, `radii` the corner
    // radius at each, and `start` a point on the edge from corner 0 to corner 1
    // where s = 0.
    Track(const std::vector<std::array<double, 2>>& corners, const std::vector<double>& radii,
          const std::array<double, 2>& start);

    double curvature(double s) const;
    // World x, y and track heading at arc length `s`, offset `e_y` to the left.
    Pose pose(double s, double e_y = 0.0) const;
    // Track coordinates (s, e_y) of the world point (x, y): the nearest centreline point.
    std::pair<double, double> project(double x, double y) const;
    // Distance travelled from the start line to `s` (0..length) along the line offset by `e_y`.
    double offset_length(double s, double e_y) const;
    // Inverse of `offset_length`, wrapping round the lap.
    double offset_to_s(double distance, double e_y) const;

    double length;
    std::vector<double> seg_length, seg_kappa;
    std::vector<double> seg_s;  // start of each piece, then the lap length
    std::vector<Pose> seg_pose;

private:
    std::pair<int, double> piece(double s) const;  // index of the piece at s and the distance into it
    std::vector<double> cumulative(double e_y) const;
};

}  // namespace ov
