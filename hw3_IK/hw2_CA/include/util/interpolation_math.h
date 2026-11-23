#pragma once

#include <functional>

#include "Eigen/Core"
#include "Eigen/Geometry"

namespace util {

// Quaternion-native interpolation signature used by the per-frame pipeline.
// All four surrounding keyframe quaternions are provided so that higher-order
// methods (Squad) can compute smooth tangents.
// Inputs are pre-hemisphere-normalised (done once at keyframe build time).
// Returns an interpolated quaternion — no Euler conversion, no gimbal lock.
using RotationInterpQuat = std::function<Eigen::Quaterniond(
    const Eigen::Quaterniond& qm1, const Eigen::Quaterniond& q0,
    const Eigen::Quaterniond& q1,  const Eigen::Quaterniond& q2,
    double t)>;

// Callable signature for position interpolation.
// Receives the four surrounding keyframe positions so that higher-order
// methods (Catmull-Rom / cubic Bezier) can estimate smooth tangents.
//   km1, k2  — positions of keyframes before k0 / after k1
//   k0,  k1  — start and end keyframe positions for this segment
//   t        — interpolation parameter in [0, 1]
using PositionInterp = std::function<Eigen::Vector4d(
    const Eigen::Vector4d& km1, const Eigen::Vector4d& k0,
    const Eigen::Vector4d& k1,  const Eigen::Vector4d& k2,
    double t)>;

// Static collection of interpolation strategies.
// To add a new method: implement a static function with the matching signature
// and pass it (or a lambda wrapping it) to bakeInterpolation.
class InterpolationMath {
 public:
    // Slerp in quaternion space (qm1/q2 unused, accepted to match signature).
    static Eigen::Quaterniond slerpQuat(
        const Eigen::Quaterniond& qm1, const Eigen::Quaterniond& q0,
        const Eigen::Quaterniond& q1,  const Eigen::Quaterniond& q2,
        double t);

    // Squad in quaternion space — C1-continuous tangent-smooth interpolation.
    static Eigen::Quaterniond squadQuat(
        const Eigen::Quaterniond& qm1, const Eigen::Quaterniond& q0,
        const Eigen::Quaterniond& q1,  const Eigen::Quaterniond& q2,
        double t);

    // Cubic Bezier whose control points are derived from Catmull-Rom tangents,
    // giving C1-continuous motion across keyframe boundaries.
    static Eigen::Vector4d RootPositionInterpolation(
        const Eigen::Vector4d& km1, const Eigen::Vector4d& k0,
        const Eigen::Vector4d& k1,  const Eigen::Vector4d& k2,
        double t);
    // Bonus: simple linear root interpolation baseline.
    static Eigen::Vector4d RootPositionLinear(
        const Eigen::Vector4d& km1, const Eigen::Vector4d& k0,
        const Eigen::Vector4d& k1,  const Eigen::Vector4d& k2,
        double t);

 private:
    // Evaluate a cubic Bezier curve B(t) given four control points.
    static Eigen::Vector4d cubicBezier(
        const Eigen::Vector4d& c0, const Eigen::Vector4d& c1,
        const Eigen::Vector4d& c2, const Eigen::Vector4d& c3,
        double t);

    // Logarithm of a unit quaternion: log(q) = theta * n  (pure quaternion).
    static Eigen::Quaterniond quaternionLog(const Eigen::Quaterniond& q);

    // Exponential of a pure quaternion p = (0, v): exp(p) = (cos|v|, sin|v|*v/|v|).
    static Eigen::Quaterniond quaternionExp(const Eigen::Quaterniond& p);

    // Compute the Squad inner control quaternion at q given neighbours qprev and qnext:
    //   s = q * exp( -(log(q^{-1}*qnext) + log(q^{-1}*qprev)) / 4 )
    static Eigen::Quaterniond squadInner(
        const Eigen::Quaterniond& qprev,
        const Eigen::Quaterniond& q,
        const Eigen::Quaterniond& qnext);

    static Eigen::Quaterniond slerp(
    const Eigen::Quaterniond& q0, const Eigen::Quaterniond& q1, double t);
};

}  // namespace util
