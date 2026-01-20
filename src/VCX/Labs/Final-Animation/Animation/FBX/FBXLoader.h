#pragma once
#include <string>
#include "FBXMesh.h"
#include "FBXSkeleton.h"
#include "FBXSkin.h"

struct FBXAsset {
    FBXMesh mesh;
    FBXSkeleton skeleton;
    FBXSkin skin;
};

class FBXLoader {
public:
    static FBXAsset Load(const std::string& path);
};
