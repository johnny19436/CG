#include "acclaim/motion.h"
#include <fstream>
#include <iostream>
#include <utility>

#include "simulation/kinematics.h"
#include "util/helper.h"

namespace acclaim {
Motion::Motion(const util::fs::path &amc_file, std::unique_ptr<Skeleton> &&_skeleton) noexcept
    : skeleton(std::move(_skeleton)) {
    postures.reserve(1024);
    if (!this->readAMCFile(amc_file)) {
        std::cerr << "Error in reading AMC file, this object is not initialized!" << std::endl;
        std::cerr << "You can call readAMCFile() to initialize again" << std::endl;
        postures.resize(0);
    }
}

const std::unique_ptr<Skeleton> &Motion::getSkeleton() const { return skeleton; }

Motion::Motion(const Motion &other) noexcept
    : skeleton(std::make_unique<Skeleton>(*other.skeleton)), postures(other.postures) {}

Motion::Motion(Motion &&other) noexcept : skeleton(std::move(other.skeleton)), postures(std::move(other.postures)) {}

Motion &Motion::operator=(const Motion &other) noexcept {
    if (this != &other) {
        skeleton.reset();
        skeleton = std::make_unique<Skeleton>(*other.skeleton);
        postures = other.postures;
    }
    return *this;
}

Motion &Motion::operator=(Motion &&other) noexcept {
    if (this != &other) {
        skeleton = std::move(other.skeleton);
        postures = std::move(other.postures);
    }
    return *this;
}

Motion::Motion(std::vector<Posture> bakedPostures, std::unique_ptr<Skeleton> sk) noexcept
    : skeleton(std::move(sk)), postures(std::move(bakedPostures)) {}

int Motion::getFrameNum() const { return static_cast<int>(postures.size()); }

const std::vector<Posture> &Motion::getPostures() const { return postures; }

void Motion::forwardkinematics(int frame_idx) {
    const Posture& posture = postures[frame_idx];
    kinematics::FrameQuat frame;
    frame.boneRotations    = posture.bone_rotations;  // already quaternions from readAMCFile
    frame.boneTranslations = posture.bone_translations;
    frame.rootTranslation  = posture.bone_translations[0];
    kinematics::forwardSolver(frame, skeleton->getBonePointer(0));
    skeleton->setModelMatrices();
}

bool Motion::readAMCFile(const util::fs::path &file_name) {
    // Open AMC file
    std::ifstream input_stream(file_name);
    // Check if file successfully opened
    if (!input_stream) {
        std::cerr << "Failed to open " << file_name << std::endl;
        return false;
    }
    // There are (NUM_BONES_IN_ASF_FILE - 2) moving bones and 2 dummy bones (lhipjoint and rhipjoint)
    int movable_bones = skeleton->getMovableBoneNum();
    // Ignore header
    input_stream.ignore(1024, '\n');
    input_stream.ignore(1024, '\n');
    input_stream.ignore(1024, '\n');
    int frame_num;
    std::string bone_name;
    while (input_stream >> frame_num) {
        auto &&current_posture = postures.emplace_back(skeleton->getBoneNum());
        for (int i = 0; i < movable_bones; ++i) {
            input_stream >> bone_name;
            const Bone &bone = *skeleton->getBonePointer(bone_name);
            int bone_idx = bone.idx;
            Eigen::Vector4d bone_rotation = Eigen::Vector4d::Zero();
            Eigen::Vector4d bone_translation = Eigen::Vector4d::Zero();
            if (bone.doftx) {
                input_stream >> bone_translation[0];
            }
            if (bone.dofty) {
                input_stream >> bone_translation[1];
            }
            if (bone.doftz) {
                input_stream >> bone_translation[2];
            }
            if (bone.dofrx) {
                input_stream >> bone_rotation[0];
            }
            if (bone.dofry) {
                input_stream >> bone_rotation[1];
            }
            if (bone.dofrz) {
                input_stream >> bone_rotation[2];
            }
            // #TODO Part 1
            // Convert the Euler-degree rotation to a unit quaternion.
            // HINT:
            // 1. util/helper.cpp
            // 2. be aware of the order of the three axes
            ///////////////////////
            Eigen::Quaterniond local_rotation_quat = util::rotateDegreeZYX(bone_rotation);
            local_rotation_quat.normalize();
            current_posture.bone_rotations[bone_idx] = local_rotation_quat;
            ///////////////////////
            current_posture.bone_translations[bone_idx] = std::move(bone_translation);
            if (bone_idx == 0) {
                current_posture.bone_translations[bone_idx] *= skeleton->getScale();
            }
        }
    }
    input_stream.close();
    std::cout << frame_num << " samples in " << file_name.string() << " are read" << std::endl;
    return true;
}
void Motion::render(graphics::Program *program) const { skeleton->render(program); }
}  // namespace acclaim
