#pragma once

#include <string>
#include <vector>

#include <glm/vec3.hpp>

namespace VCX::Labs::FinalAnimation::BVH {
    enum class ChannelType {Xposition, Yposition, Zposition, Xrotation, Yrotation, Zrotation};
    struct Joint {
        std::string name;
        int parent = -1;
        glm::vec3 offset{0.0f}; 
        std::vector<ChannelType> channels;
    };

    struct BVHData {
        std::vector<Joint> joints; // all joints in depth-first order
        int channelCount = 0;
        int frameCount = 0;
        float frameTime = 0.0f;
        std::vector<std::vector<float>> motion; // motion data
    };
}
