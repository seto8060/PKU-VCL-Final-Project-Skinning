#pragma once

#include "Labs/Final-Animation/Animation/BVH/BVH.h"
#include "Labs/Final-Animation/Animation/Pose/Skeleton.h"
#include "Labs/Final-Animation/Animation/Pose/Pose.h"

namespace VCX::Labs::FinalAnimation {
    LocalPose BuildLocalPose(const BVH::BVHData& bvh, const Skeleton& skeleton, int frameIndex);
    LocalPose BuildLocalPose_Frame0(const Skeleton& skeleton);
}