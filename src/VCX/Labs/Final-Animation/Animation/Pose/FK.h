#pragma once
#include "Skeleton.h"
#include "Pose.h"

namespace VCX::Labs::FinalAnimation {
    class FK {
    public:
        static GlobalPose Compute(const Skeleton& skeleton, const LocalPose& localPose);
    };
}