// Planar geometry shared by the scenarios.
#pragma once

#include <Eigen/Dense>
#include <cmath>

namespace ov {

using Corners = Eigen::Matrix<double, 4, 2>;  // corners of a car body, one per row

// Remainder with the sign of the divisor, as Python's % operator.
inline double pymod(double a, double b) {
    const double r = std::fmod(a, b);
    return (r != 0.0 && (r < 0.0) != (b < 0.0)) ? r + b : r;
}

// Angle wrapped to [-pi, pi).
inline double wrap_angle(double angle) { return pymod(angle + M_PI, 2.0 * M_PI) - M_PI; }

// Signed distance from `from` to `to` round a lap of length `lap`, in [-lap/2, lap/2).
inline double lap_difference(double to, double from, double lap) { return pymod(to - from + 0.5 * lap, lap) - 0.5 * lap; }

// Corners of a rectangle of the given length and width centred on (x, y) with heading psi.
Corners body_corners(double x, double y, double psi, double length, double width);

// Distance between two convex polygons, 0 if they overlap.
double polygon_distance(const Corners& P, const Corners& Q);

}  // namespace ov
