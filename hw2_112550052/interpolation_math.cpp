#include "util/interpolation_math.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "util/helper.h"

namespace util {

// =============================================================================
// Public quaternion-native rotation interpolators (no Euler conversion)
// =============================================================================

Eigen::Quaterniond InterpolationMath::slerpQuat(
    const Eigen::Quaterniond& /*qm1*/, const Eigen::Quaterniond& q0,
    const Eigen::Quaterniond& q1,      const Eigen::Quaterniond& /*q2*/,
    double t) {

    return slerp(q0, q1, t);
}


Eigen::Quaterniond InterpolationMath::slerp(
    const Eigen::Quaterniond& q0, const Eigen::Quaterniond& q1, double t) {

    // #TODO Part 5 - slerp
    t = std::clamp(t, 0.0, 1.0);
    Eigen::Quaterniond startQuat = q0.normalized();
    Eigen::Quaterniond endQuat = q1.normalized();
    double cosOmega = startQuat.dot(endQuat);

    // Hemisphere fix: choose the shorter arc on S^3.
    if (cosOmega < 0.0) {
        endQuat.coeffs() *= -1.0;
        cosOmega = -cosOmega;
    }
    cosOmega = std::clamp(cosOmega, -1.0, 1.0);

    constexpr double kNlerpThreshold = 1.0 - 1e-8;
    if (cosOmega > kNlerpThreshold) {
        Eigen::Quaterniond blendedQuat;
        blendedQuat.coeffs() = (1.0 - t) * startQuat.coeffs() + t * endQuat.coeffs();
        blendedQuat.normalize();
        return blendedQuat;
    }

    const double omega = std::acos(cosOmega);
    const double sinOmega = std::sin(omega);
    const double startWeight = std::sin((1.0 - t) * omega) / sinOmega;
    const double endWeight = std::sin(t * omega) / sinOmega;

    Eigen::Quaterniond resultQuat;
    resultQuat.coeffs() = startWeight * startQuat.coeffs() + endWeight * endQuat.coeffs();
    resultQuat.normalize();
    return resultQuat;
}

Eigen::Quaterniond InterpolationMath::squadQuat(
    const Eigen::Quaterniond& qm1, const Eigen::Quaterniond& q0,
    const Eigen::Quaterniond& q1,  const Eigen::Quaterniond& q2,
    double t) {

    // #TODO Part 5 - Squad
    // Inputs are hemisphere-normalised at build time.
    // Compute inner control quaternions and evaluate Squad.
    t = std::clamp(t, 0.0, 1.0);
    const Eigen::Quaterniond firstArc = slerp(q0, q1, t);
    // Basic control heuristic from neighboring keys.
    const Eigen::Quaterniond controlStart = slerp(qm1, q1, 0.5);
    const Eigen::Quaterniond controlEnd = slerp(q0, q2, 0.5);
    const Eigen::Quaterniond controlArc = slerp(controlStart, controlEnd, t);
    const double blendT = 2.0 * t * (1.0 - t);
    return slerp(firstArc, controlArc, blendT);
}

// =============================================================================
// Public position interpolators
// =============================================================================

Eigen::Vector4d InterpolationMath::RootPositionInterpolation(
    const Eigen::Vector4d& km1, const Eigen::Vector4d& k0,
    const Eigen::Vector4d& k1,  const Eigen::Vector4d& k2,
    double t) {

    // #TODO Part 4
    // Root position interpolation
    ///////////////////////
    t = std::clamp(t, 0.0, 1.0);

    const Eigen::Vector4d tangentAtStart = 0.5 * (k1 - km1);
    const Eigen::Vector4d tangentAtEnd = 0.5 * (k2 - k0);

    const Eigen::Vector4d bezierP0 = k0;
    const Eigen::Vector4d bezierP1 = k0 + tangentAtStart / 3.0;
    const Eigen::Vector4d bezierP2 = k1 - tangentAtEnd / 3.0;
    const Eigen::Vector4d bezierP3 = k1;

    return cubicBezier(bezierP0, bezierP1, bezierP2, bezierP3, t);
}

Eigen::Vector4d InterpolationMath::RootPositionLinear(
    const Eigen::Vector4d& /*km1*/, const Eigen::Vector4d& k0,
    const Eigen::Vector4d& k1,      const Eigen::Vector4d& /*k2*/,
    double t) {
    t = std::clamp(t, 0.0, 1.0);
    return (1.0 - t) * k0 + t * k1;
}

Eigen::Vector4d InterpolationMath::cubicBezier(
    const Eigen::Vector4d& c0, const Eigen::Vector4d& c1,
    const Eigen::Vector4d& c2, const Eigen::Vector4d& c3,
    double t) {
    t = std::clamp(t, 0.0, 1.0);
    const double oneMinusT = 1.0 - t;
    const double basis0 = oneMinusT * oneMinusT * oneMinusT;
    const double basis1 = 3.0 * oneMinusT * oneMinusT * t;
    const double basis2 = 3.0 * oneMinusT * t * t;
    const double basis3 = t * t * t;
    return basis0 * c0 + basis1 * c1 + basis2 * c2 + basis3 * c3;
}

}  // namespace util
