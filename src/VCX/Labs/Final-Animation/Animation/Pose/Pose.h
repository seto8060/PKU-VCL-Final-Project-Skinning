#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <vector>
// What a frame is like
namespace VCX::Labs::FinalAnimation {
    struct LocalPose {
        std::vector<glm::vec3> translation;
        std::vector<glm::quat> rotation;
    };
    struct GlobalPose {
        std::vector<glm::vec3> position;
        std::vector<glm::mat4> transform;
    };
}