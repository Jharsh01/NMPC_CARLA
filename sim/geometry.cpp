#include "sim/geometry.hpp"

#include <algorithm>
#include <limits>

namespace ov {

Corners body_corners(double x, double y, double psi, double length, double width) {
    const double hl = 0.5 * length, hw = 0.5 * width;
    const double c = std::cos(psi), s = std::sin(psi);
    Corners local;
    local << hl, hw, -hl, hw, -hl, -hw, hl, -hw;
    Eigen::Matrix2d rotation;
    rotation << c, s, -s, c;
    Corners out = local * rotation;
    out.rowwise() += Eigen::RowVector2d(x, y);
    return out;
}

// Separating-axis test for two convex polygons.
static bool overlap(const Corners& P, const Corners& Q) {
    for (const Corners* A : {&P, &Q}) {
        for (int i = 0; i < 4; ++i) {
            const Eigen::RowVector2d edge = A->row(i) - A->row((i + 3) % 4);
            const Eigen::Vector2d normal(-edge[1], edge[0]);
            const Eigen::Vector4d p = P * normal, q = Q * normal;
            if (p.maxCoeff() < q.minCoeff() || q.maxCoeff() < p.minCoeff()) return false;
        }
    }
    return true;
}

static double point_segment_distance(const Eigen::RowVector2d& a, const Eigen::RowVector2d& b0,
                                     const Eigen::RowVector2d& b1) {
    const Eigen::RowVector2d d = b1 - b0;
    const double s = std::clamp((a - b0).dot(d) / d.dot(d), 0.0, 1.0);
    return (a - b0 - s * d).norm();
}

double polygon_distance(const Corners& P, const Corners& Q) {
    if (overlap(P, Q)) return 0.0;
    double best = std::numeric_limits<double>::infinity();
    for (const auto& [A, B] : {std::pair{&P, &Q}, std::pair{&Q, &P}})
        for (int a = 0; a < 4; ++a)
            for (int i = 0; i < 4; ++i)
                best = std::min(best, point_segment_distance(A->row(a), B->row(i), B->row((i + 3) % 4)));
    return best;
}

}  // namespace ov
