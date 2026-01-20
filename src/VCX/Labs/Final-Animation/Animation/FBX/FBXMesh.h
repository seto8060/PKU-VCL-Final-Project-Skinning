#pragma once
#include <vector>
#include <glm/glm.hpp>
#include <cstdint>

struct FBXMesh {
    std::vector<glm::vec3> positions;
    std::vector<glm::vec3> normals;
    std::vector<uint32_t> indices;
    std::vector<glm::vec3> skinnedPositions;
};
