#pragma once
#include <vector>
#include <string>
#include <glm/glm.hpp>

struct FBXJoint {
    std::string name;
    int parent = -1;
    glm::mat4 bindPose = glm::mat4(1.0f), inverseBindPose = glm::mat4(1.0f);
};

struct FBXSkeleton {
    std::vector<FBXJoint> joints;
};
