#pragma once
#include <vector>
#include <glm/glm.hpp>
#include "FBXSkin.h"
#include "Labs/Final-Animation/Animation/Pose/Pose.h"

namespace VCX::Labs::FinalAnimation {
    struct DualQuat {
        glm::quat real;
        glm::quat dual;
    };
    void SkinCPU(const std::vector<glm::vec3>& restPositions, const FBXSkin& skin, const GlobalPose& globalPose, std::vector<glm::vec3>& outSkinnedPositions, bool useDualQuat);
}