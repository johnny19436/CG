#pragma once
#include <vector>

#include "Eigen/Geometry"

namespace acclaim {
struct Bone;
}
namespace kinematics {

// Holds one frame of animation data in quaternion form.
// Used for both keyframe-native data and Posture conversions.
struct FrameQuat {
    std::vector<Eigen::Quaterniond> boneRotations;    // local rotation per bone
    Eigen::Vector4d rootTranslation;                  // world position of root
    std::vector<Eigen::Vector4d> boneTranslations;    // per-bone translations (non-root usually zero)
};

// Apply forward kinematics to the skeleton.
// Three-step pipeline:
//   Step 1  Caller fills FrameQuat (from keyframe data or Posture conversion)
//   Step 2  Local Transform — bone->rot_parent_current * local_rotation (per bone)
//   Step 3  FK             — parent->rotation * local_transform (propagated up the chain)
void forwardSolver(const FrameQuat& frame, acclaim::Bone* bone);

}  // namespace kinematics
