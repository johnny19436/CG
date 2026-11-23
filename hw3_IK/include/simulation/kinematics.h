#pragma once
#include <vector>
#include <Eigen/Geometry>
#include "acclaim/posture.h"

namespace acclaim {
struct Bone;
}
namespace kinematics {
enum class IKAlgorithm { Jacobian = 0, CCD = 1 };

using JointChain = std::vector<Eigen::Vector4d*>;
using JointChains = std::vector<JointChain>;
using BoneChain = std::vector<acclaim::Bone*>;
using BoneChains = std::vector<BoneChain>;

// Apply forward kinematics to skeleton
void forwardSolver(const acclaim::Posture& posture, acclaim::Bone* bone);

bool inverseIKSolver(const std::vector<Eigen::Vector4d>& targets, acclaim::Bone* end_bone, acclaim::Posture& posture,
                     JointChains& jointChains, BoneChains& boneChains, const Eigen::Vector4d& currentBasePos,
                     IKAlgorithm algorithm, bool enableConvergenceTest);

// Bonus: runtime IK tuning (defaults match the original constants).
double getJacobianIKStepSize();
double getCCDIKStepSize();
double getJacobianSvdDamping();
double getIKSmallSolveDamping();
void setJacobianIKStepSize(double step);
void setCCDIKStepSize(double step);
void setJacobianSvdDamping(double lambda);
void setIKSmallSolveDamping(double lambda);
void resetIKParametersToDefault();
}  // namespace kinematics
