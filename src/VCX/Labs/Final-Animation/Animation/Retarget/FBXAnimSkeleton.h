#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <vector>
#include <string>

#include "Labs/Final-Animation/Animation/Pose/Skeleton.h"
#include "Labs/Final-Animation/Animation/FBX/FBXSkeleton.h"
#include "Labs/Final-Animation/Animation/Pose/Pose.h"

namespace VCX::Labs::FinalAnimation {
    struct FBXAnimSkeletonBuildResult {
        Skeleton skeleton;
        std::vector<glm::quat> bindLocalRot;
        std::vector<int> rawToAnim;
        std::vector<int> animToRaw; // mapping
    };

    class FBXAnimSkeleton {
    public:
        static FBXAnimSkeletonBuildResult Build(const FBXSkeleton& raw);
        static FBXAnimSkeletonBuildResult Build(const FBXSkeleton& raw, const std::vector<int>& selectedRawJoints);

        static LocalPose MakeBindPoseLocalPose(const FBXAnimSkeletonBuildResult& built);
        static std::string BaseName(const std::string& name);

    private:
        static glm::vec3 ExtractTranslation(const glm::mat4& m);
        static glm::quat ExtractRotationOrthonormalized(const glm::mat4& m);
        
        static bool IsFingerLike(const std::string& base);
        static bool IsMainSemanticJoint(const std::string& base);
        static std::vector<int> SelectSemanticJoints(const FBXSkeleton& raw);
    };
}
