#include "simulation/kinematics.h"

#include <algorithm>
#include <cmath>
#include "Eigen/Dense"
#include "Eigen/SVD"
#include "acclaim/bone.h"
#include "util/helper.h"

namespace kinematics {
namespace internal {
double jacobian_ik_step = 0.3;
double ccd_ik_step = 0.3;
double jacobian_svd_damping = 1e-6;
double small_solve_damping = 1e-4;
}  // namespace internal

namespace {
constexpr double kConvergeEpsilon = 1e-3;
constexpr double kTiny = 1e-6;
constexpr int kMaxIteration = 1000;

double clampScalar(double v, double lo, double hi) { return std::min(std::max(v, lo), hi); }

/**
 * @brief Required context to perform IK on chain.
 *
 * @param target_pos 
 * @param start_bone This bone is the last bone you can move while doing IK
 * @param posture The original AMC motion's reference, you need to modify this
 * @param jointChains A 2D vector containing multiple 1D vectors, each of which holds pointers to Eigen::Vector4d
 * constituting a chain.
 * @param boneChains A 2D vector containing multiple 1D vectors, each of which holds pointers to acclaim::Bone
 * constituting a chain.
 * @param currentBasePos The base of the current chain.
 */

struct IKChainContext {
    Eigen::Vector4d target;                 // The position where `end_bone` (idx = 0 in the chain) will move to.
    acclaim::Bone* rootBone;                // Pointer to the root bone.
    acclaim::Posture& posture;              // The original AMC motion's reference, you need to modify this.
    JointChain jointChain;                  // 1D vector which holds pointers to Eigen::Vector4d constituting a chain.
    BoneChain boneChain;                    // 1D vector which holds pointers to acclaim::Bone constituting a chain.
    const Eigen::Vector4d& currentBasePos;  // The base of the current chain.
    int chainIdx = 0;                       // Chain index of the current chain.
    int boneNum = 0;                        // Number of bones in the chain.
};

/**
 * @brief Returns true if all end effectors reaches corresponding targets, otherwise false.
 */
bool IKConverged(const std::vector<Eigen::Vector4d>& targets, JointChains& jointChains) {
    // TODO: Test if all end effectors reaches target.
    const int n = static_cast<int>(jointChains.size());
    if (n == 0 || static_cast<int>(targets.size()) < n) {
        return false;
    }
    for (int i = 0; i < n; ++i) {
        if (jointChains[i].empty()) {
            return false;
        }
        const Eigen::Vector3d err = targets[i].head<3>() - jointChains[i][0]->head<3>();
        if (err.norm() > kConvergeEpsilon) {
            return false;
        }
    }
    return true;
}

Eigen::Vector4d clampBoneRotation(Eigen::Vector4d rotation, const acclaim::Bone* bone) {
    // TODO: Clamp the rotation based on the bone's rotation limit.
    if (bone->dofrx) {
        rotation.x() = clampScalar(static_cast<double>(rotation.x()), static_cast<double>(bone->rxmin),
                                   static_cast<double>(bone->rxmax));
    }
    if (bone->dofry) {
        rotation.y() = clampScalar(static_cast<double>(rotation.y()), static_cast<double>(bone->rymin),
                                   static_cast<double>(bone->rymax));
    }
    if (bone->dofrz) {
        rotation.z() = clampScalar(static_cast<double>(rotation.z()), static_cast<double>(bone->rzmin),
                                   static_cast<double>(bone->rzmax));
    }
    return rotation;
}

Eigen::VectorXd pseudoInverseLinearSolver(const Eigen::Matrix4Xd& Jacobian, const Eigen::Vector4d& target) {
    // TODO (find x which min(| jacobian * x - target |))
    // Hint:
    //   1. Linear algebra - least squares solution
    //   2. https://en.wikipedia.org/wiki/Moore%E2%80%93Penrose_inverse#Construction
    // Note:
    //   1. SVD or other pseudo-inverse method is useful
    //   2. Some of them have some limitation, if you use that method you should check it.
    const int n = static_cast<int>(Jacobian.cols());
    if (n == 0) {
        return Eigen::VectorXd();
    }
    Eigen::MatrixXd J = Jacobian;
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const Eigen::VectorXd& S = svd.singularValues();
    const int rank = static_cast<int>(S.size());
    constexpr double kSingularTol = 1e-8;
    const double damp = internal::jacobian_svd_damping;
    Eigen::VectorXd Sinv(rank);
    for (int i = 0; i < rank; ++i) {
        const double s = S(i);
        if (s > kSingularTol) {
            Sinv(i) = s / (s * s + damp * damp);
        } else {
            Sinv(i) = 0.0;
        }
    }
    return svd.matrixV() * Sinv.asDiagonal() * svd.matrixU().transpose() * target;
}

// Damped least-squares solve for A x = b (A is m x n, b is m-vector).
Eigen::VectorXd dampedLeastSquaresSolve(const Eigen::MatrixXd& A, const Eigen::VectorXd& b) {
    const int n = static_cast<int>(A.cols());
    if (n == 0 || A.rows() != b.rows()) {
        return Eigen::VectorXd();
    }
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const Eigen::VectorXd& S = svd.singularValues();
    const int r = static_cast<int>(S.size());
    constexpr double kSingularTol = 1e-8;
    const double damp = internal::small_solve_damping;
    Eigen::VectorXd Sinv(r);
    for (int i = 0; i < r; ++i) {
        const double s = S(i);
        Sinv(i) = s > kSingularTol ? s / (s * s + damp * damp) : 0.0;
    }
    return svd.matrixV() * Sinv.asDiagonal() * svd.matrixU().transpose() * b;
}

void JacobianChainSolver(IKChainContext& chain) {
    // TODO: Implement Jocobian solver

    for (int iter = 0; iter < kMaxIteration; ++iter) {
        forwardSolver(chain.posture, chain.rootBone);

        const Eigen::Vector4d& ee = *chain.jointChain[0];
        const Eigen::Vector3d err = chain.target.head<3>() - ee.head<3>();
        // Get vector from target to chain end bone and break early if convergence is reached.
        if (err.norm() < kConvergeEpsilon) {
            break;
        }

        const Eigen::Vector3d p = ee.head<3>();
        int dofCount = 0;
        for (int bi = 0; bi < chain.boneNum; ++bi) {
            const acclaim::Bone* b = chain.boneChain[bi];
            if (b->dofrx) {
                ++dofCount;
            }
            if (b->dofry) {
                ++dofCount;
            }
            if (b->dofrz) {
                ++dofCount;
            }
        }
        if (dofCount == 0) {
            break;
        }

        // Construct Jacobian.
        Eigen::Matrix4Xd J(4, dofCount);
        J.setZero();
        std::vector<std::pair<acclaim::Bone*, int>> dofMeta;
        dofMeta.reserve(static_cast<size_t>(dofCount));

        int col = 0;
        for (int bi = 0; bi < chain.boneNum; ++bi) {
            acclaim::Bone* bone = chain.boneChain[bi];
            const Eigen::Vector3d r = bone->start_position.head<3>();
            const Eigen::Vector3d v = p - r;

            const Eigen::Vector4d& rotDeg = chain.posture.bone_rotations[bone->idx];
            const double ry = util::toRadian(rotDeg.y());
            const double rz = util::toRadian(rotDeg.z());
            const Eigen::Matrix3d Rz = Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ()).toRotationMatrix();
            const Eigen::Matrix3d Ry = Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY()).toRotationMatrix();
            const Eigen::Matrix3d Rzy = Rz * Ry;

            Eigen::Matrix3d R_pc_world;
            if (bone->parent == nullptr) {
                R_pc_world.setIdentity();
            } else {
                R_pc_world = (bone->parent->rotation * bone->rot_parent_current).linear();
            }

            const Eigen::Vector3d axis_x = R_pc_world * Rzy * Eigen::Vector3d::UnitX();
            const Eigen::Vector3d axis_y = R_pc_world * Rz * Eigen::Vector3d::UnitY();
            const Eigen::Vector3d axis_z = R_pc_world * Eigen::Vector3d::UnitZ();

            auto pushCol = [&](const Eigen::Vector3d& axis) {
                const Eigen::Vector3d colVec = axis.cross(v);
                J(0, col) = colVec.x();
                J(1, col) = colVec.y();
                J(2, col) = colVec.z();
                J(3, col) = 0.0;
                ++col;
            };

            if (bone->dofrx) {
                dofMeta.emplace_back(bone, 0);
                pushCol(axis_x);
            }
            if (bone->dofry) {
                dofMeta.emplace_back(bone, 1);
                pushCol(axis_y);
            }
            if (bone->dofrz) {
                dofMeta.emplace_back(bone, 2);
                pushCol(axis_z);
            }
        }

        Eigen::Vector4d targetStep;
        targetStep.head<3>() = err * internal::jacobian_ik_step;
        targetStep.w() = 0.0;

        // Solve Jacobian.
        const Eigen::VectorXd deltaRad = pseudoInverseLinearSolver(J, targetStep);
        if (deltaRad.size() != dofCount) {
            break;
        }

        // Apply and clamp rotation.
        for (int i = 0; i < dofCount; ++i) {
            acclaim::Bone* bone = dofMeta[static_cast<size_t>(i)].first;
            const int axis = dofMeta[static_cast<size_t>(i)].second;
            const double dDeg = deltaRad(i) * 180.0 / util::PI;
            if (axis == 0) {
                chain.posture.bone_rotations[bone->idx].x() += dDeg;
            } else if (axis == 1) {
                chain.posture.bone_rotations[bone->idx].y() += dDeg;
            } else {
                chain.posture.bone_rotations[bone->idx].z() += dDeg;
            }
            chain.posture.bone_rotations[bone->idx] =
                clampBoneRotation(chain.posture.bone_rotations[bone->idx], bone);
        }

        // Fix Root
        if (chain.chainIdx == 0) {
            chain.posture.bone_translations[0] += (chain.currentBasePos - *chain.jointChain[chain.boneNum]);
        }

        // Apply FK
        forwardSolver(chain.posture, chain.rootBone);
    }
}

void CCDChainSolver(IKChainContext& chain) {
    // TODO: Implement CCD solver

    for (int iter = 0; iter < kMaxIteration; ++iter) {
        forwardSolver(chain.posture, chain.rootBone);

        const Eigen::Vector4d& ee = *chain.jointChain[0];
        const Eigen::Vector3d err = chain.target.head<3>() - ee.head<3>();
        // Get vector from target to chain end bone and break early if convergence is reached.
        if (err.norm() < kConvergeEpsilon) {
            break;
        }

        // CCD: adjust from the joint near the end effector toward the base.
        for (int bi = chain.boneNum - 1; bi >= 0; --bi) {
            acclaim::Bone* bone = chain.boneChain[bi];
            if (!bone->dofrx && !bone->dofry && !bone->dofrz) {
                continue;
            }

            const Eigen::Vector3d pivot = bone->start_position.head<3>();
            const Eigen::Vector4d eeNow = *chain.jointChain[0];
            Eigen::Vector3d ve = eeNow.head<3>() - pivot;
            Eigen::Vector3d vt = chain.target.head<3>() - pivot;
            // Get target and end effector vectors
            const double lenE = ve.norm();
            const double lenT = vt.norm();
            if (lenE < kTiny || lenT < kTiny) {
                continue;
            }
            ve /= lenE;
            vt /= lenT;

            // Calculate world rotation delta (shortest rotation taking ve toward vt, scaled by step size)
            Eigen::Quaterniond q_align = Eigen::Quaterniond::FromTwoVectors(ve, vt);
            q_align.normalize();
            Eigen::AngleAxisd aa_world(q_align);
            double angle = aa_world.angle();
            if (angle < kTiny) {
                continue;
            }
            angle *= internal::ccd_ik_step;
            const Eigen::Vector3d omega_world = aa_world.axis() * angle;

            const Eigen::Vector4d& rotDeg = chain.posture.bone_rotations[bone->idx];
            const double ry = util::toRadian(rotDeg.y());
            const double rz = util::toRadian(rotDeg.z());
            const Eigen::Matrix3d Rz = Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ()).toRotationMatrix();
            const Eigen::Matrix3d Ry = Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY()).toRotationMatrix();
            const Eigen::Matrix3d Rzy = Rz * Ry;

            Eigen::Matrix3d R_pc_world;
            if (bone->parent == nullptr) {
                R_pc_world.setIdentity();
            } else {
                R_pc_world = (bone->parent->rotation * bone->rot_parent_current).linear();
            }

            const Eigen::Vector3d axis_x = R_pc_world * Rzy * Eigen::Vector3d::UnitX();
            const Eigen::Vector3d axis_y = R_pc_world * Rz * Eigen::Vector3d::UnitY();
            const Eigen::Vector3d axis_z = R_pc_world * Eigen::Vector3d::UnitZ();

            int dofBone = 0;
            if (bone->dofrx) {
                ++dofBone;
            }
            if (bone->dofry) {
                ++dofBone;
            }
            if (bone->dofrz) {
                ++dofBone;
            }
            Eigen::MatrixXd A(3, dofBone);
            int col = 0;
            if (bone->dofrx) {
                A.col(col++) = axis_x;
            }
            if (bone->dofry) {
                A.col(col++) = axis_y;
            }
            if (bone->dofrz) {
                A.col(col++) = axis_z;
            }

            // Project world rotation delta to bone coordinates (match instantaneous ZYX Euler axes, damped LS)
            const Eigen::VectorXd dRad = dampedLeastSquaresSolve(A, omega_world);
            if (dRad.size() != dofBone) {
                continue;
            }

            // Clamp and apply bone rotation
            col = 0;
            if (bone->dofrx) {
                chain.posture.bone_rotations[bone->idx].x() += dRad(col++) * 180.0 / util::PI;
            }
            if (bone->dofry) {
                chain.posture.bone_rotations[bone->idx].y() += dRad(col++) * 180.0 / util::PI;
            }
            if (bone->dofrz) {
                chain.posture.bone_rotations[bone->idx].z() += dRad(col++) * 180.0 / util::PI;
            }
            chain.posture.bone_rotations[bone->idx] =
                clampBoneRotation(chain.posture.bone_rotations[bone->idx], bone);

            // Calculate the actual applied rotation in world space
            // (post-clamp pose is reflected in the next FK; no separate analytic delta stored)

            // Update cachedEffectorPos (effector world position lives in jointChain[0])
            forwardSolver(chain.posture, chain.rootBone);
        }

        // Fix Root
        if (chain.chainIdx == 0) {
            chain.posture.bone_translations[0] += (chain.currentBasePos - *chain.jointChain[chain.boneNum]);
        }

        // Apply FK
        forwardSolver(chain.posture, chain.rootBone);
    }
}

}  // namespace

bool inverseIKSolver(const std::vector<Eigen::Vector4d>& targets, acclaim::Bone* endBone, acclaim::Posture& posture,
                     JointChains& jointChains, BoneChains& boneChains, const Eigen::Vector4d& currentBasePos,
                     IKAlgorithm algorithm, bool enableConvergenceTest) {
    acclaim::Bone* rootBone = endBone - endBone->idx;
    acclaim::Posture originalPosture(posture);
    // Ensure world bone poses match posture before updating shared chains.
    forwardSolver(posture, rootBone);
    for (int chainIdx = 0; chainIdx < static_cast<int>(boneChains.size()); ++chainIdx) {
        IKChainContext chainContext{
            targets[chainIdx],    rootBone,       posture,  jointChains[chainIdx],
            boneChains[chainIdx], currentBasePos, chainIdx, static_cast<int>(boneChains[chainIdx].size())};
        if (algorithm == IKAlgorithm::CCD) {
            CCDChainSolver(chainContext);
        } else {
            JacobianChainSolver(chainContext);
        }
    }

    if (enableConvergenceTest && !IKConverged(targets, jointChains)) {
        posture = originalPosture;
        forwardSolver(posture, rootBone);
        return false;
    }
    return true;
}

void forwardSolver(const acclaim::Posture& posture, acclaim::Bone* bone) {
    while (bone != nullptr) {
        Eigen::Quaterniond local_rotation = util::rotateDegreeZYX(posture.bone_rotations[bone->idx]);
        local_rotation.normalize();
        Eigen::Affine3d rot_local;
        rot_local = local_rotation.toRotationMatrix();

        if (bone->parent == nullptr) {
            bone->rotation = rot_local;
            bone->start_position = bone->end_position = posture.bone_translations[0];
        } else {
            bone->rotation = bone->parent->rotation * bone->rot_parent_current * rot_local;
            bone->start_position = bone->end_position = bone->parent->end_position;
            bone->end_position += posture.bone_translations[bone->idx];
            bone->end_position += bone->rotation * bone->dir * bone->length;
        }
        forwardSolver(posture, bone->child);
        bone = bone->sibling;
    }
}
double getJacobianIKStepSize() { return internal::jacobian_ik_step; }
double getCCDIKStepSize() { return internal::ccd_ik_step; }
double getJacobianSvdDamping() { return internal::jacobian_svd_damping; }
double getIKSmallSolveDamping() { return internal::small_solve_damping; }

void setJacobianIKStepSize(double step) { internal::jacobian_ik_step = std::max(1e-6, step); }
void setCCDIKStepSize(double step) { internal::ccd_ik_step = std::max(1e-6, step); }
void setJacobianSvdDamping(double lambda) { internal::jacobian_svd_damping = std::max(1e-12, lambda); }
void setIKSmallSolveDamping(double lambda) { internal::small_solve_damping = std::max(1e-12, lambda); }

void resetIKParametersToDefault() {
    internal::jacobian_ik_step = 0.3;
    internal::ccd_ik_step = 0.3;
    internal::jacobian_svd_damping = 1e-6;
    internal::small_solve_damping = 1e-4;
}
}  // namespace kinematics
