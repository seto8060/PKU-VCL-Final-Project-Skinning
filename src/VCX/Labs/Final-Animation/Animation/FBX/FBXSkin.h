#pragma once
#include <vector>
#include <array>
#include <cstdint>

struct SkinWeight {
    uint32_t joint = 0;
    float weight = 0.0f;
};

struct FBXSkin {
    std::vector<std::vector<SkinWeight>> weights;
    std::vector<glm::mat4> invBindGlobal;
};
