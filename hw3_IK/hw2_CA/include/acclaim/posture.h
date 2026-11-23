#pragma once

#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "Eigen/StdVector"

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector4d)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Quaterniond)

namespace acclaim {
struct Posture final {
    Posture() noexcept;
    explicit Posture(const std::size_t size) noexcept;
    Posture(const Posture &) noexcept;
    Posture(Posture &&) noexcept;

    Posture &operator=(const Posture &) noexcept;
    Posture &operator=(Posture &&) noexcept;
    // You need this for alignment otherwise it may crash
    // Ref: https://eigen.tuxfamily.org/dox/group__TopicStructHavingEigenMembers.html
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Rotations are stored as unit quaternions (converted from Euler at load time).
    std::vector<Eigen::Quaterniond> bone_rotations;
    std::vector<Eigen::Vector4d> bone_translations;
};
}  // namespace acclaim
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(acclaim::Posture)
