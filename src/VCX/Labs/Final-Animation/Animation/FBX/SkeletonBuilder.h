#pragma once

#include "FBXLoader.h"
#include <vector>

struct BuiltSkeleton {
    FBXSkeleton skeleton;
    std::vector<int> rawToAnim;
};

class SkeletonBuilder {
public:
    static BuiltSkeleton BuildFullSkeleton(const FBXSkeleton& raw);
};
