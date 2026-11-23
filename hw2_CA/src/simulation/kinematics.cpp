#include "simulation/kinematics.h"

#include "acclaim/bone.h"

namespace kinematics {

namespace {

void propagateBonePose(const FrameQuat& frameData, acclaim::Bone* currentBone) {
    if (currentBone == nullptr) return;

    const int boneIndex = currentBone->idx;
    const Eigen::Affine3d localJointRotation =
        currentBone->rot_parent_current * frameData.boneRotations[boneIndex];

    if (currentBone->parent == nullptr) {
        currentBone->start_position = frameData.rootTranslation;
    } else {
        currentBone->start_position = currentBone->parent->end_position;
    }
    currentBone->start_position[3] = 1.0;

    if (currentBone->parent == nullptr) {
        currentBone->rotation = localJointRotation;
    } else {
        currentBone->rotation = currentBone->parent->rotation * localJointRotation;
    }

    const Eigen::Vector3d localBoneVector = currentBone->dir.head<3>() * currentBone->length;
    const Eigen::Vector3d worldBoneVector = currentBone->rotation.linear() * localBoneVector;
    currentBone->end_position = currentBone->start_position;
    currentBone->end_position.head<3>() += worldBoneVector;
    currentBone->end_position[3] = 1.0;

    propagateBonePose(frameData, currentBone->child);
    propagateBonePose(frameData, currentBone->sibling);
}

}  // namespace

void forwardSolver(const FrameQuat& frame, acclaim::Bone* bone) {
    // #TODO Part 2: Forward Kinematic
    // Hint:
    // - Traverse the skeleton tree from root to leaves.
    // - Compute each bone's global rotation and global position.
    // - Use local rotation (from posture) and bone hierarchy (parent rotation, offset, etc).
    // - Remember to update both bone->start_position and bone->end_position.
    // - Use bone->rotation to store global rotation (after combining parent, local, etc).
    // - Reminder - the data in frame.boneRotations are already Quaterniond not Euler 
    propagateBonePose(frame, bone);
}

}  // namespace kinematics
