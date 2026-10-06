#include "sim/scenarios/circuit/track.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "sim/geometry.hpp"

namespace ov {

// Pose after travelling `ds` from (x, y, psi) at constant curvature.
static Pose advance(const Pose& from, double kappa, double ds) {
    const double end = from.psi + kappa * ds;
    if (kappa == 0.0) return {from.x + ds * std::cos(from.psi), from.y + ds * std::sin(from.psi), end};
    return {from.x + (std::sin(end) - std::sin(from.psi)) / kappa,
            from.y - (std::cos(end) - std::cos(from.psi)) / kappa, end};
}

// Linear interpolation of the points (xp, fp) at x, clamped at the ends (as numpy.interp).
static double interp(double x, const std::vector<double>& xp, const std::vector<double>& fp) {
    if (x <= xp.front()) return fp.front();
    if (x >= xp.back()) return fp.back();
    const size_t i = std::upper_bound(xp.begin(), xp.end(), x) - xp.begin();
    return fp[i - 1] + (x - xp[i - 1]) / (xp[i] - xp[i - 1]) * (fp[i] - fp[i - 1]);
}

Track::Track(const std::vector<std::array<double, 2>>& corners, const std::vector<double>& radii,
             const std::array<double, 2>& start) {
    const int n = static_cast<int>(corners.size());
    std::vector<double> edge_len(n), heading(n), turn(n), tangent(n), straight(n);
    for (int i = 0; i < n; ++i) {  // edge i runs from corner i to corner i + 1
        const double dx = corners[(i + 1) % n][0] - corners[i][0], dy = corners[(i + 1) % n][1] - corners[i][1];
        edge_len[i] = std::hypot(dx, dy);
        heading[i] = std::atan2(dy, dx);
    }
    for (int i = 0; i < n; ++i) {
        turn[i] = wrap_angle(heading[i] - heading[(i + n - 1) % n]);  // heading change at corner i
        tangent[i] = radii[i] * std::tan(0.5 * std::fabs(turn[i]));  // corner to arc end
    }
    for (int i = 0; i < n; ++i) {
        straight[i] = edge_len[i] - tangent[i] - tangent[(i + 1) % n];
        if (straight[i] < -1e-9) throw std::invalid_argument("corner radii do not fit on the edges");
    }
    const double to_start = std::hypot(start[0] - corners[0][0], start[1] - corners[0][1]) - tangent[0];
    if (!(0.0 <= to_start && to_start <= straight[0]))
        throw std::invalid_argument("start must lie on the straight part of the first edge");

    // (length, curvature) from the start line round to the start line
    auto add = [this](double piece_length, double kappa) {
        if (piece_length > 1e-9) {
            seg_length.push_back(piece_length);
            seg_kappa.push_back(kappa);
        }
    };
    add(straight[0] - to_start, 0.0);
    for (int k = 1; k <= n; ++k) {
        const int i = k % n;
        add(radii[i] * std::fabs(turn[i]), (turn[i] > 0.0 ? 1.0 : turn[i] < 0.0 ? -1.0 : 0.0) / radii[i]);
        add(i ? straight[i] : to_start, 0.0);
    }

    seg_s.assign(1, 0.0);
    for (double piece_length : seg_length) seg_s.push_back(seg_s.back() + piece_length);
    length = seg_s.back();

    Pose at{start[0], start[1], heading[0]};
    for (size_t k = 0; k < seg_length.size(); ++k) {
        seg_pose.push_back(at);
        at = advance(at, seg_kappa[k], seg_length[k]);
    }
    if (std::hypot(at.x - start[0], at.y - start[1]) > 1e-6) throw std::invalid_argument("track does not close");
}

std::pair<int, double> Track::piece(double s) const {
    s = pymod(s, length);
    const int last = static_cast<int>(seg_length.size()) - 1;
    const int k = std::clamp(static_cast<int>(std::upper_bound(seg_s.begin(), seg_s.end(), s) - seg_s.begin()) - 1,
                             0, last);
    return {k, s - seg_s[k]};
}

double Track::curvature(double s) const { return seg_kappa[piece(s).first]; }

Pose Track::pose(double s, double e_y) const {
    const auto [k, ds] = piece(s);
    const Pose at = advance(seg_pose[k], seg_kappa[k], ds);
    return {at.x - e_y * std::sin(at.psi), at.y + e_y * std::cos(at.psi), at.psi};
}

std::pair<double, double> Track::project(double x, double y) const {
    double best = std::numeric_limits<double>::infinity(), best_s = 0.0, best_e = 0.0;
    for (size_t k = 0; k < seg_length.size(); ++k) {
        const Pose& p0 = seg_pose[k];
        const double kappa = seg_kappa[k], piece_length = seg_length[k];
        double ds;
        if (kappa == 0.0) {
            ds = (x - p0.x) * std::cos(p0.psi) + (y - p0.y) * std::sin(p0.psi);
        } else {
            // angle swept round the arc's centre, measured in the direction of travel
            const double cx = p0.x - std::sin(p0.psi) / kappa, cy = p0.y + std::cos(p0.psi) / kappa;
            const double swept = (kappa > 0.0 ? 1.0 : -1.0) *
                                 wrap_angle(std::atan2(y - cy, x - cx) - std::atan2(p0.y - cy, p0.x - cx));
            ds = swept / std::fabs(kappa);
            // nearer the far end, the other way round
            if (ds < -0.5 * (2.0 * M_PI / std::fabs(kappa) - piece_length)) ds += 2.0 * M_PI / std::fabs(kappa);
        }
        ds = std::clamp(ds, 0.0, piece_length);
        const Pose q = advance(p0, kappa, ds);
        const double e_y = -(x - q.x) * std::sin(q.psi) + (y - q.y) * std::cos(q.psi);
        const double dist = std::hypot(x - q.x, y - q.y);
        if (dist < best) {
            best = dist;
            best_s = seg_s[k] + ds;
            best_e = e_y;
        }
    }
    return {pymod(best_s, length), best_e};
}

std::vector<double> Track::cumulative(double e_y) const {
    std::vector<double> out(1, 0.0);
    for (size_t k = 0; k < seg_length.size(); ++k)  // an inside line is shorter
        out.push_back(out.back() + seg_length[k] * (1.0 - seg_kappa[k] * e_y));
    return out;
}

double Track::offset_length(double s, double e_y) const { return interp(s, seg_s, cumulative(e_y)); }

double Track::offset_to_s(double distance, double e_y) const {
    const std::vector<double> c = cumulative(e_y);
    return interp(pymod(distance, c.back()), c, seg_s);
}

}  // namespace ov
