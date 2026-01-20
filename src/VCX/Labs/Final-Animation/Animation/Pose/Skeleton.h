#pragma once
#include <glm/glm.hpp>
#include <vector>
#include <string>
// The part that don't move
namespace VCX::Labs::FinalAnimation {
    struct Joint {
        std::string name;
        int parent = -1;
        glm::vec3 offset{0.0f};
    };

    class Skeleton {
    public:
        std::vector<Joint> joints;
        int GetJointCount() const { return static_cast<int>(joints.size()); }
    };
}