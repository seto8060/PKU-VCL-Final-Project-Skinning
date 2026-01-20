#pragma once
#include <vector>
#include <string>
#include <unordered_map>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "Labs/Final-Animation/Animation/Pose/Skeleton.h"
#include "Labs/Final-Animation/Animation/Pose/Pose.h"

namespace VCX::Labs::FinalAnimation {

    class Retargeter {
    public:
        struct MapPair {int bvh = -1, fbx = -1;};

        Retargeter(const Skeleton& bvhSkel,
                const Skeleton& fbxAnimSkel,
                const std::vector<glm::quat>& fbxBindLocalRot);

        void BuildDefaultMapping_MixamoLike(); // A -> B mapping

        // compute semantic-local offset
        void ComputeSemanticLocalOffsets(const LocalPose& bvhRefPose, const LocalPose& fbxBindPose);

        // retarget: only change mapped joints' local rotation; keep unmapped joints' bind
        LocalPose RetargetFrame(const LocalPose& bvhPose, float rootTranslationScale, const LocalPose& fbxBindPose) const;

        int FindBVH(const std::string& name) const;
        int FindFBX(const std::string& name) const;

    private:
        void AddMapByName(const std::string& bvhName, const std::string& fbxName);

    private:
        const Skeleton& _bvh;
        const Skeleton& _fbx;
        const std::vector<glm::quat>& _fbxBindLocalRot;

        std::unordered_map<std::string, int> _bvhNameToIndex;
        std::unordered_map<std::string, int> _fbxNameToIndex;

        std::vector<MapPair> _mapping;

        std::vector<glm::quat> _semOffsets;

        std::vector<int> _fbxToBvh;

        int _fbxHips = -1;
        int _fbxLeftHipPivot = -1;
        int _fbxRightHipPivot = -1;

    private:
        static glm::quat SafeNormalize(const glm::quat& q);

        // get global transform's rotation
        static glm::quat RotFromMat4(const glm::mat4& M);

        // get local pose's rotation (already quaternion)
        static glm::quat RotFromLocal(const glm::quat& q);

        // find the nearest mapped ancestor joint of fbx joint f
        int ResolveBvhSemanticParentForFbx(int fbxIdx) const;

        // compute BVH's semantic-local: R_sem = inv(G_parent_sem) * G_self
        glm::quat ComputeBvhSemanticLocal(const std::vector<glm::mat4>& bvhGlobal, int bvhIdx, int bvhSemanticParent) const;

        // compute FBX's local: R_local = inv(G_parent) * G_self
        glm::quat ComputeFbxLocalFromGlobal(const std::vector<glm::mat4>& fbxGlobal, int fbxIdx) const;
    };
}
