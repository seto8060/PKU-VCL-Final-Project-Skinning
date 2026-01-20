#include "FK.h"

namespace VCX::Labs::FinalAnimation {

    GlobalPose FK::Compute(const Skeleton& skeleton, const LocalPose& localPose) {
        GlobalPose globalPose;
        const int jointCount = skeleton.GetJointCount();
        globalPose.position.resize(jointCount);
        globalPose.transform.resize(jointCount);
        for (int i = 0; i < jointCount; i++){
            const Joint& joint = skeleton.joints[i];
            //cal the local matrix
            glm::mat4 localMat;
            if (joint.parent == -1){
                localMat = glm::translate(glm::mat4(1.0f), localPose.translation[i]) * glm::mat4_cast(localPose.rotation[i]);
            } else {
                localMat = glm::translate(glm::mat4(1.0f), joint.offset) * glm::mat4_cast(localPose.rotation[i]);
            }
            //cal the global matrix
            globalPose.transform[i] = (joint.parent == -1) ? localMat : globalPose.transform[joint.parent] * localMat;
            globalPose.position[i] = glm::vec3(globalPose.transform[i][3]);
        }
        return globalPose;
    }
}