#include "BuildLocalPose.h"
#define GLM_ENABLE_EXPERIMENTAL

#include<glm/gtc/quaternion.hpp>
#include<glm/gtx/quaternion.hpp>
#include <iostream>

namespace VCX::Labs::FinalAnimation {
    LocalPose BuildLocalPose(const BVH::BVHData& bvh, const Skeleton& skeleton, int frameIndex = 0) {
        LocalPose localPose;
        const int jointCount = skeleton.GetJointCount();
        localPose.translation.resize(jointCount);
        localPose.rotation.resize(jointCount);

        const auto& frames = bvh.motion[frameIndex];
        int channelIndex = 0;
        for (int i = 0; i < jointCount; i++){
            const auto& skeletonJoint = skeleton.joints[i];
            const auto& bvhJoint = bvh.joints[i];
            glm::vec3 T(0.0f); glm::quat R(1.0f, 0.0f, 0.0f, 0.0f);

            for (const auto& channel : bvhJoint.channels){
                float value = frames[channelIndex++];
                switch (channel){
                    case BVH::ChannelType::Xposition: T.x = value; break;
                    case BVH::ChannelType::Yposition: T.y = value; break;
                    case BVH::ChannelType::Zposition: T.z = value; break;
                    case BVH::ChannelType::Xrotation: R = R * glm::angleAxis(glm::radians(value), glm::vec3(1.0f, 0.0f, 0.0f)); break;
                    case BVH::ChannelType::Yrotation: R = R * glm::angleAxis(glm::radians(value), glm::vec3(0.0f, 1.0f, 0.0f)); break;
                    case BVH::ChannelType::Zrotation: R = R * glm::angleAxis(glm::radians(value), glm::vec3(0.0f, 0.0f, 1.0f)); break;
                }
            }
            if (bvhJoint.parent != -1) T = skeletonJoint.offset;
            localPose.translation[i] = T;
            localPose.rotation[i] = R;
        }
        return localPose;
    }
    LocalPose BuildLocalPose_Frame0(const Skeleton& skeleton) {
        LocalPose localPose;
        const int jointCount = skeleton.GetJointCount();
        localPose.translation.resize(jointCount);
        localPose.rotation.resize(jointCount);
        for (int i = 0; i < jointCount; i++) localPose.translation[i] = glm::vec3(0.0f), localPose.rotation[i] = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
        return localPose;
    }
}